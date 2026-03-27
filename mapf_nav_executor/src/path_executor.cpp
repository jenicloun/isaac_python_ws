#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_stamped.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mapf_msgs/msg/global_plan.hpp"
#include "mapf_msgs/msg/single_plan.hpp"

using std::placeholders::_1;

class PathExecutor : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Nav2ActionClient = rclcpp_action::Client<NavigateToPose>;

  PathExecutor()
  : Node("path_executor"),
    make_span_(0),
    has_new_plan_(false),
    pose_initialized_(false),
    stop_requested_(false)
  {
    declare_parameter<int>("agent_num", 3);
    declare_parameter<std::string>("global_plan_topic", "/mapf/global_plan");
    declare_parameter<std::string>("global_frame_id", "map");
    declare_parameter<double>("xy_goal_tolerance", 0.2);
    declare_parameter<double>("yaw_goal_tolerance", 0.2);
    declare_parameter<double>("intermediate_goal_tolerance", 0.4);
    declare_parameter<bool>("cancel_previous_goal_on_new_plan", true);
    declare_parameter<double>("pose_update_rate", 10.0);
    declare_parameter<double>("executor_rate", 10.0);

    get_parameter("agent_num", agent_num_);
    get_parameter("global_plan_topic", global_plan_topic_);
    get_parameter("global_frame_id", global_frame_id_);
    get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
    get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
    get_parameter("intermediate_goal_tolerance", intermediate_goal_tolerance_);
    get_parameter("cancel_previous_goal_on_new_plan", cancel_previous_goal_on_new_plan_);
    get_parameter("pose_update_rate", pose_update_rate_);
    get_parameter("executor_rate", executor_rate_);

    agent_names_.resize(agent_num_);
    base_frame_ids_.resize(agent_num_);
    current_poses_.resize(agent_num_);
    last_sent_plans_.resize(agent_num_);
    plan_arr_.resize(agent_num_);
    action_clients_.resize(agent_num_);
    active_goal_handles_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      declare_parameter<std::string>(
        "agent_name.agent_" + std::to_string(i),
        "carter" + std::to_string(i + 1));
      declare_parameter<std::string>(
        "base_frame_id.agent_" + std::to_string(i),
        "carter" + std::to_string(i + 1) + "/chassis_link/base_link");

      get_parameter("agent_name.agent_" + std::to_string(i), agent_names_[i]);
      get_parameter("base_frame_id.agent_" + std::to_string(i), base_frame_ids_[i]);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    for (int i = 0; i < agent_num_; ++i) {
      const std::string action_name = "/" + agent_names_[i] + "/navigate_to_pose";
      action_clients_[i] = rclcpp_action::create_client<NavigateToPose>(this, action_name);
      RCLCPP_INFO(get_logger(), "Agent %d action client: %s", i, action_name.c_str());
    }

    sub_global_plan_ = create_subscription<mapf_msgs::msg::GlobalPlan>(
      global_plan_topic_, 10, std::bind(&PathExecutor::planCallback, this, _1));

    pose_thread_ = std::make_unique<std::thread>(&PathExecutor::poseThread, this);
    exec_thread_ = std::make_unique<std::thread>(&PathExecutor::execThread, this);

    RCLCPP_INFO(get_logger(), "PathExecutor initialized.");
  }

  ~PathExecutor() override
  {
    stop_requested_ = true;
    if (pose_thread_ && pose_thread_->joinable()) {
      pose_thread_->join();
    }
    if (exec_thread_ && exec_thread_->joinable()) {
      exec_thread_->join();
    }
  }

private:
  int agent_num_;
  int make_span_;

  std::string global_plan_topic_;
  std::string global_frame_id_;

  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double intermediate_goal_tolerance_;
  double pose_update_rate_;
  double executor_rate_;

  bool cancel_previous_goal_on_new_plan_;
  bool has_new_plan_;
  bool pose_initialized_;
  bool stop_requested_;

  std::mutex plan_mtx_;

  std::vector<std::string> agent_names_;
  std::vector<std::string> base_frame_ids_;
  std::vector<geometry_msgs::msg::PoseStamped> current_poses_;
  std::vector<mapf_msgs::msg::SinglePlan> last_sent_plans_;
  std::vector<mapf_msgs::msg::SinglePlan> plan_arr_;

  rclcpp::Subscription<mapf_msgs::msg::GlobalPlan>::SharedPtr sub_global_plan_;

  std::unique_ptr<std::thread> pose_thread_;
  std::unique_ptr<std::thread> exec_thread_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<Nav2ActionClient::SharedPtr> action_clients_;
  std::vector<GoalHandleNavigateToPose::SharedPtr> active_goal_handles_;

  double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  bool sameSinglePlan(
    const mapf_msgs::msg::SinglePlan & a,
    const mapf_msgs::msg::SinglePlan & b,
    double eps = 1e-6)
  {
    if (a.time_step.size() != b.time_step.size()) {
      return false;
    }
    if (a.plan.poses.size() != b.plan.poses.size()) {
      return false;
    }

    for (size_t i = 0; i < a.plan.poses.size(); ++i) {
      const auto & pa = a.plan.poses[i].pose;
      const auto & pb = b.plan.poses[i].pose;

      if (std::fabs(pa.position.x - pb.position.x) > eps ||
          std::fabs(pa.position.y - pb.position.y) > eps ||
          std::fabs(pa.position.z - pb.position.z) > eps ||
          std::fabs(pa.orientation.x - pb.orientation.x) > eps ||
          std::fabs(pa.orientation.y - pb.orientation.y) > eps ||
          std::fabs(pa.orientation.z - pb.orientation.z) > eps ||
          std::fabs(pa.orientation.w - pb.orientation.w) > eps)
      {
        return false;
      }
    }
    return true;
  }

  bool nearToCurGoal(
    const geometry_msgs::msg::PoseStamped & cur_pose,
    const geometry_msgs::msg::PoseStamped & cur_goal,
    double xy_tolerance,
    double yaw_tolerance = 2.0 * M_PI)
  {
    const double dx = cur_pose.pose.position.x - cur_goal.pose.position.x;
    const double dy = cur_pose.pose.position.y - cur_goal.pose.position.y;

    const double cur_yaw = yawFromQuat(cur_pose.pose.orientation);
    const double goal_yaw = yawFromQuat(cur_goal.pose.orientation);
    const double dyaw = std::fabs(cur_yaw - goal_yaw);

    const double dist_sq = dx * dx + dy * dy;

    return dyaw < yaw_tolerance &&
           dist_sq < (xy_tolerance * xy_tolerance) &&
           dist_sq > 1e-8;
  }

  geometry_msgs::msg::PoseStamped normalizeGoalPose(
    const geometry_msgs::msg::PoseStamped & in_pose)
  {
    geometry_msgs::msg::PoseStamped out = in_pose;
    out.header.frame_id = global_frame_id_;
    out.header.stamp = now();
    return out;
  }

  NavigateToPose::Goal toNavGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    NavigateToPose::Goal goal;
    goal.pose = normalizeGoalPose(pose);
    return goal;
  }

  void cancelActiveGoal(int agent_idx)
  {
    if (!active_goal_handles_[agent_idx]) {
      return;
    }

    action_clients_[agent_idx]->async_cancel_goal(active_goal_handles_[agent_idx]);
    RCLCPP_INFO(get_logger(), "Agent %d cancel requested for previous goal", agent_idx);
    active_goal_handles_[agent_idx].reset();
  }

  void poseThread()
  {
    RCLCPP_INFO(get_logger(), "pose_thread: Getting current poses...");
    rclcpp::Rate loop_rate(pose_update_rate_);

    while (rclcpp::ok() && !stop_requested_) {
      for (int i = 0; i < agent_num_; ++i) {
        try {
          geometry_msgs::msg::PoseStamped robot_pose;
          robot_pose.header.frame_id = base_frame_ids_[i];
          robot_pose.header.stamp = rclcpp::Time(0);
          robot_pose.pose.orientation.w = 1.0;

          tf_buffer_->transform(robot_pose, current_poses_[i], global_frame_id_);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Failed to transform pose for agent %d: %s", i, ex.what());
        }
      }

      if (!pose_initialized_) {
        bool ok = true;
        for (int i = 0; i < agent_num_; ++i) {
          if (current_poses_[i].header.frame_id.empty()) {
            ok = false;
            break;
          }
        }
        if (ok) {
          pose_initialized_ = true;
          RCLCPP_INFO(get_logger(), "INITIALIZE POSE DONE.");
        }
      }

      loop_rate.sleep();
    }
  }

  void planCallback(const mapf_msgs::msg::GlobalPlan::SharedPtr msg)
  {
    if (static_cast<int>(msg->global_plan.size()) != agent_num_) {
      RCLCPP_WARN(
        get_logger(),
        "Received global plan agent count mismatch: expected=%d got=%zu",
        agent_num_, msg->global_plan.size());
      return;
    }

    std::lock_guard<std::mutex> lock(plan_mtx_);

    make_span_ = msg->makespan;
    bool any_changed = false;

    RCLCPP_INFO(
      get_logger(),
      "Received new MAPF global plan. makespan=%d, agents=%zu",
      msg->makespan, msg->global_plan.size());

    for (int i = 0; i < agent_num_; ++i) {
      if (!sameSinglePlan(last_sent_plans_[i], msg->global_plan[i])) {
        plan_arr_[i] = msg->global_plan[i];
        any_changed = true;
      } else {
        RCLCPP_INFO(get_logger(), "Agent %d path unchanged. Skip resend.", i);
      }
    }

    if (any_changed) {
      has_new_plan_ = true;
      for (int i = 0; i < agent_num_; ++i) {
        last_sent_plans_[i] = msg->global_plan[i];
      }
    }
  }

  void execThread()
  {
    RCLCPP_INFO(get_logger(), "path_executor_thread: waiting for MAPF plans...");
    rclcpp::Rate loop_rate(executor_rate_);

    while (rclcpp::ok() && !stop_requested_) {
      loop_rate.sleep();

      if (!pose_initialized_) {
        continue;
      }

      if (!has_new_plan_) {
        continue;
      }

      std::unique_lock<std::mutex> lock(plan_mtx_);
      has_new_plan_ = false;

      auto local_plans = plan_arr_;
      const int local_makespan = make_span_;
      lock.unlock();

      for (int i = 0; i < agent_num_; ++i) {
        if (cancel_previous_goal_on_new_plan_) {
          cancelActiveGoal(i);
        }
      }

      for (int step = 0; step < local_makespan && rclcpp::ok() && !stop_requested_; ++step) {
        bool interrupted = false;

        for (int agent = 0; agent < agent_num_; ++agent) {
          if (step >= static_cast<int>(local_plans[agent].plan.poses.size())) {
            continue;
          }

          if (!action_clients_[agent]->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Agent %d action server not available", agent);
            continue;
          }

          const auto goal_pose = normalizeGoalPose(local_plans[agent].plan.poses[step]);

          RCLCPP_INFO(
            get_logger(),
            "Agent %d send step %d goal (x=%.3f, y=%.3f, frame=%s)",
            agent, step,
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            goal_pose.header.frame_id.c_str());

          auto send_goal_options = Nav2ActionClient::SendGoalOptions();

          send_goal_options.goal_response_callback =
            [this, agent](GoalHandleNavigateToPose::SharedPtr goal_handle)
            {
              if (!goal_handle) {
                RCLCPP_WARN(this->get_logger(), "Agent %d goal rejected", agent);
              } else {
                active_goal_handles_[agent] = goal_handle;
                RCLCPP_INFO(this->get_logger(), "Agent %d goal accepted", agent);
              }
            };

          send_goal_options.result_callback =
            [this, agent](const GoalHandleNavigateToPose::WrappedResult & result)
            {
              switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                  RCLCPP_INFO(this->get_logger(), "Agent %d FollowPath succeeded", agent);
                  break;
                case rclcpp_action::ResultCode::ABORTED:
                  RCLCPP_WARN(this->get_logger(), "Agent %d FollowPath aborted", agent);
                  break;
                case rclcpp_action::ResultCode::CANCELED:
                  RCLCPP_WARN(this->get_logger(), "Agent %d FollowPath canceled", agent);
                  break;
                default:
                  RCLCPP_WARN(this->get_logger(), "Agent %d FollowPath unknown result", agent);
                  break;
              }
            };

          action_clients_[agent]->async_send_goal(toNavGoal(goal_pose), send_goal_options);
        }

        loop_rate.sleep();

        for (int agent = 0; agent < agent_num_ && rclcpp::ok() && !stop_requested_; ++agent) {
          if (step >= static_cast<int>(local_plans[agent].plan.poses.size())) {
            continue;
          }

          const auto goal_pose = normalizeGoalPose(local_plans[agent].plan.poses[step]);
          const bool is_last = (step == static_cast<int>(local_plans[agent].plan.poses.size()) - 1);

          while (rclcpp::ok() && !stop_requested_) {
            const double xy_tol = is_last ? xy_goal_tolerance_ : intermediate_goal_tolerance_;
            const double yaw_tol = is_last ? yaw_goal_tolerance_ : 2.0 * M_PI;

            if (nearToCurGoal(current_poses_[agent], goal_pose, xy_tol, yaw_tol)) {
              RCLCPP_INFO(
                get_logger(),
                "Agent %d reached step %d%s",
                agent, step, is_last ? " (END)" : "");
              break;
            }

            {
              std::lock_guard<std::mutex> relock(plan_mtx_);
              if (has_new_plan_) {
                interrupted = true;
                RCLCPP_INFO(get_logger(), "Plan changed during execution. Restarting...");
                break;
              }
            }

            loop_rate.sleep();
          }

          if (interrupted) {
            break;
          }
        }

        if (interrupted) {
          break;
        }
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}