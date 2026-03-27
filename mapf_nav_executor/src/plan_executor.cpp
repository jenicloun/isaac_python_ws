#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "mapf_msgs/msg/global_plan.hpp"
#include "mapf_msgs/msg/single_plan.hpp"

using std::placeholders::_1;

class ParamServer : public rclcpp::Node
{
public:
  int agent_num_;
  std::vector<std::string> agent_name_;
  std::string global_frame_id_;
  std::vector<std::string> base_frame_id_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double step_wait_timeout_sec_;

  explicit ParamServer(const std::string & node_name)
  : Node(node_name)
  {
    this->declare_parameter<double>("xy_goal_tolerance", 0.25);
    this->declare_parameter<double>("yaw_goal_tolerance", 0.35);
    this->declare_parameter<double>("step_wait_timeout", 20.0);
    this->declare_parameter<int>("agent_num", 1);
    this->declare_parameter<std::string>("global_frame_id", "map");

    this->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
    this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
    this->get_parameter("step_wait_timeout", step_wait_timeout_sec_);
    this->get_parameter("agent_num", agent_num_);
    this->get_parameter("global_frame_id", global_frame_id_);

    agent_name_.resize(agent_num_);
    base_frame_id_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      this->declare_parameter<std::string>(
        "base_frame_id.agent_" + std::to_string(i), "base_link");
      this->declare_parameter<std::string>(
        "agent_name.agent_" + std::to_string(i), "agent_" + std::to_string(i));

      this->get_parameter("base_frame_id.agent_" + std::to_string(i), base_frame_id_[i]);
      this->get_parameter("agent_name.agent_" + std::to_string(i), agent_name_[i]);
    }
  }
};

class PlanExecutor : public ParamServer
{
private:
  std::mutex plan_mtx_;

  rclcpp::Subscription<mapf_msgs::msg::GlobalPlan>::SharedPtr sub_mapf_plan_;

  int make_span_;
  std::vector<mapf_msgs::msg::SinglePlan> plan_arr_;

  bool get_plan_;
  bool pose_initialize_;
  bool stop_threads_;

  std::vector<geometry_msgs::msg::PoseStamped> cur_poses_;
  std::vector<geometry_msgs::msg::PoseStamped> last_goals_;

  std::unique_ptr<std::thread> planner_thread_;
  std::unique_ptr<std::thread> get_pose_thread_;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleNavigateToPoseFuture =
    std::shared_future<std::shared_ptr<GoalHandleNavigateToPose>>;
  using Nav2ActionClient = rclcpp_action::Client<NavigateToPose>;

  std::vector<Nav2ActionClient::SharedPtr> ac_ptr_arr_;

  std::shared_ptr<tf2_ros::TransformListener> tf_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

public:
  PlanExecutor()
  : ParamServer("plan_executor"),
    make_span_(0),
    get_plan_(false),
    pose_initialize_(false),
    stop_threads_(false)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    plan_arr_.resize(agent_num_);
    ac_ptr_arr_.resize(agent_num_);
    cur_poses_.resize(agent_num_);
    last_goals_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      const std::string action_name = "/" + agent_name_[i] + "/navigate_to_pose";
      ac_ptr_arr_[i] = rclcpp_action::create_client<NavigateToPose>(this, action_name);
      RCLCPP_INFO(this->get_logger(), "Agent %d action client: %s", i, action_name.c_str());
    }

    sub_mapf_plan_ = this->create_subscription<mapf_msgs::msg::GlobalPlan>(
      "global_plan", 1, std::bind(&PlanExecutor::planCallback, this, _1));

    get_pose_thread_ = std::make_unique<std::thread>(&PlanExecutor::getPoseThread, this);
    planner_thread_ = std::make_unique<std::thread>(&PlanExecutor::mbStateThread, this);

    rclcpp::Rate loop_rate(10.0);
    while (rclcpp::ok() && !pose_initialize_) {
      loop_rate.sleep();
    }

    last_goals_ = cur_poses_;
    RCLCPP_INFO(this->get_logger(), "PlanExecutor initialized.");
  }

  ~PlanExecutor() override
  {
    stop_threads_ = true;

    if (get_pose_thread_ && get_pose_thread_->joinable()) {
      get_pose_thread_->join();
    }
    if (planner_thread_ && planner_thread_->joinable()) {
      planner_thread_->join();
    }
  }

void getPoseThread()
{
  RCLCPP_INFO(this->get_logger(), "get_pose_thread: Getting current poses...");
  rclcpp::Rate loop_rate(10.0);

  while (rclcpp::ok() && !stop_threads_) {
    loop_rate.sleep();

    bool all_ok = true;

    for (int i = 0; i < agent_num_; ++i) {
      try {
        geometry_msgs::msg::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = base_frame_id_[i];
        robot_pose.header.stamp = rclcpp::Time(0);

        tf_buffer_->transform(robot_pose, cur_poses_[i], global_frame_id_);
      } catch (const tf2::TransformException & ex) {
        all_ok = false;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to transform pose for agent %d: %s", i, ex.what());
      }
    }

    if (!pose_initialize_ && all_ok) {
      pose_initialize_ = true;
      RCLCPP_INFO(this->get_logger(), "INITIALIZE POSE DONE.");
    }
  }
}

  void mbStateThread()
  {
    RCLCPP_INFO(this->get_logger(), "plan_executor_thread: waiting for MAPF plans...");
    rclcpp::Rate loop_rate(10.0);

    std::vector<GoalHandleNavigateToPoseFuture> send_goal_future_(agent_num_);

    while (rclcpp::ok() && !pose_initialize_) {
      loop_rate.sleep();

      if (!get_plan_) {
        continue;
      }

      std::unique_lock<std::mutex> lock(plan_mtx_);
      get_plan_ = false;

      std::vector<geometry_msgs::msg::PoseStamped> cur_goal_(agent_num_);

      for (int n = 0; n < agent_num_; ++n) {
        if (!plan_arr_[n].plan.poses.empty()) {
          last_goals_[n] = plan_arr_[n].plan.poses.back();
        }
      }

      RCLCPP_INFO(this->get_logger(), "Executing new plan. makespan=%d", make_span_);

      for (int step = 0; step < make_span_; ++step) {
        RCLCPP_INFO(this->get_logger(), "=== STEP %d ===", step);

        for (int agent = 0; agent < static_cast<int>(plan_arr_.size()); ++agent) {
          if (step >= static_cast<int>(plan_arr_[agent].time_step.size()) ||
              step >= static_cast<int>(plan_arr_[agent].plan.poses.size()))
          {
            continue;
          }

          const auto & target_pose = plan_arr_[agent].plan.poses[step];

          if (samePose(target_pose, last_goals_[agent])) {
            RCLCPP_INFO(
              this->get_logger(),
              "Agent %d step %d skipped (same as last goal).", agent, step);
            cur_goal_[agent] = target_pose;
            continue;
          }

          if (!ac_ptr_arr_[agent]->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(
              this->get_logger(),
              "Agent %d action server not available: /%s/navigate_to_pose",
              agent, agent_name_[agent].c_str());
            continue;
          }

          RCLCPP_INFO(
            this->get_logger(),
            "Agent %d send step %d goal(x, y) = (%.3f, %.3f)",
            agent, step,
            target_pose.pose.position.x,
            target_pose.pose.position.y);

          auto send_goal_options = Nav2ActionClient::SendGoalOptions();

          send_goal_options.goal_response_callback =
            [this, agent](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
              if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Agent %d goal rejected", agent);
              } else {
                RCLCPP_INFO(this->get_logger(), "Agent %d goal accepted", agent);
              }
            };

          cur_goal_[agent] = target_pose;
          auto goal_msg = getMBGoalFromGeoPose(target_pose);
          send_goal_future_[agent] = ac_ptr_arr_[agent]->async_send_goal(goal_msg, send_goal_options);
          last_goals_[agent] = target_pose;
        }

        // If a new plan arrived while sending goals, restart with the new one
        lock.unlock();
        loop_rate.sleep();
        lock.lock();

        if (get_plan_) {
          RCLCPP_WARN(this->get_logger(), "Plan changed during execution. Restarting...");
          break;
        }

        // Wait all agents reach current step goal
        RCLCPP_INFO(this->get_logger(), "Waiting agents to reach current step goals...");

        for (int agent = 0; agent < static_cast<int>(plan_arr_.size()); ++agent) {
          if (step >= static_cast<int>(plan_arr_[agent].time_step.size()) ||
              step >= static_cast<int>(plan_arr_[agent].plan.poses.size()))
          {
            continue;
          }

          const auto start_time = this->now();

          while (rclcpp::ok() && !stop_threads_) {
            loop_rate.sleep();

            const bool is_last_step =
              (step == static_cast<int>(plan_arr_[agent].time_step.size()) - 1);

            bool reached = false;
            if (is_last_step) {
              reached = nearToCurGoal(
                cur_poses_[agent], cur_goal_[agent],
                xy_goal_tolerance_, yaw_goal_tolerance_);
            } else {
              reached = nearToCurGoal(cur_poses_[agent], cur_goal_[agent], 0.4);
            }

            if (reached) {
              RCLCPP_INFO(
                this->get_logger(),
                "Agent %d reached step %d%s",
                agent, step, is_last_step ? " (END)" : "");
              break;
            }

            if ((this->now() - start_time).seconds() > step_wait_timeout_sec_) {
              RCLCPP_WARN(
                this->get_logger(),
                "Agent %d timeout waiting for step %d goal reach.",
                agent, step);
              break;
            }

            lock.unlock();
            loop_rate.sleep();
            lock.lock();

            if (get_plan_) {
              RCLCPP_WARN(this->get_logger(), "Plan changed while waiting. Restarting...");
              break;
            }
          }

          if (get_plan_) {
            break;
          }
        }

        if (get_plan_) {
          break;
        }
      }
    }
  }

  bool samePose(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b,
    double pos_eps = 1e-4,
    double yaw_eps = 1e-3)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dist2 = dx * dx + dy * dy;

    const double yaw_a = tf2::getYaw(a.pose.orientation);
    const double yaw_b = tf2::getYaw(b.pose.orientation);
    const double dyaw = std::fabs(yaw_a - yaw_b);

    return dist2 < pos_eps * pos_eps && dyaw < yaw_eps;
  }

  bool nearToCurGoal(
    const geometry_msgs::msg::PoseStamped & cur_pose,
    const geometry_msgs::msg::PoseStamped & cur_goal,
    double xy_tolerance,
    double yaw_tolerance = 2 * M_PI)
  {
    const double diff_x = cur_pose.pose.position.x - cur_goal.pose.position.x;
    const double diff_y = cur_pose.pose.position.y - cur_goal.pose.position.y;
    const double diff_yaw =
      tf2::getYaw(cur_pose.pose.orientation) - tf2::getYaw(cur_goal.pose.orientation);

    return std::fabs(diff_yaw) < yaw_tolerance &&
           (diff_x * diff_x + diff_y * diff_y) < xy_tolerance * xy_tolerance &&
           (diff_x * diff_x + diff_y * diff_y) > 1e-6;
  }

  void planCallback(const mapf_msgs::msg::GlobalPlan::SharedPtr mapf_global_plan)
  {
    if (!equal(plan_arr_, mapf_global_plan->global_plan)) {
      std::lock_guard<std::mutex> lock(plan_mtx_);

      get_plan_ = true;
      make_span_ = mapf_global_plan->makespan;

      for (int i = 0; i < agent_num_; ++i) {
        plan_arr_[i] = mapf_global_plan->global_plan[i];
      }

      RCLCPP_INFO(this->get_logger(), "Get New plan..");
      RCLCPP_INFO(this->get_logger(), "MakeSpan: %d", make_span_);
      RCLCPP_INFO(this->get_logger(), "AgentNum: %d", agent_num_);
    }
  }

  bool equal(const mapf_msgs::msg::SinglePlan & a, const mapf_msgs::msg::SinglePlan & b)
  {
    if (a.time_step.size() != b.time_step.size() ||
        a.plan.poses.size() != b.plan.poses.size())
    {
      return false;
    }

    bool res = true;
    for (size_t i = 0; i < a.plan.poses.size(); ++i) {
      res &= (
        a.plan.poses[i].pose.position.x == b.plan.poses[i].pose.position.x &&
        a.plan.poses[i].pose.position.y == b.plan.poses[i].pose.position.y &&
        a.plan.poses[i].pose.orientation.w == b.plan.poses[i].pose.orientation.w);
    }
    return res;
  }

  bool equal(
    const std::vector<mapf_msgs::msg::SinglePlan> & a,
    const std::vector<mapf_msgs::msg::SinglePlan> & b)
  {
    if (a.size() != b.size()) {
      return false;
    }

    bool res = true;
    for (size_t i = 0; i < a.size(); ++i) {
      res &= equal(a[i], b[i]);
    }
    return res;
  }

  nav2_msgs::action::NavigateToPose::Goal
  getMBGoalFromGeoPose(const geometry_msgs::msg::PoseStamped & curr_location)
  {
    nav2_msgs::action::NavigateToPose::Goal tmp_goal;
    tmp_goal.pose.header.frame_id = global_frame_id_;
    tmp_goal.pose.header.stamp = this->get_clock()->now();
    tmp_goal.pose.pose = curr_location.pose;
    return tmp_goal;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanExecutor>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}