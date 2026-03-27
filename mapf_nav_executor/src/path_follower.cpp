#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mapf_msgs/msg/global_plan.hpp"
#include "mapf_msgs/msg/single_plan.hpp"

using std::placeholders::_1;

class PathFollowerExecutor : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using FollowPathActionClient = rclcpp_action::Client<FollowPath>;

  PathFollowerExecutor()
  : Node("path_follower"),
    make_span_(0),
    has_new_plan_(false),
    stop_requested_(false)
  {
    declare_parameter<int>("agent_num", 3);
    declare_parameter<std::string>("global_plan_topic", "/mapf/global_plan");
    declare_parameter<std::string>("global_frame_id", "map");

    declare_parameter<bool>("cancel_previous_goal_on_new_plan", true);
    declare_parameter<double>("executor_rate", 5.0);

    // 이 3개 이름은 네 controller_server 설정과 맞아야 함
    declare_parameter<std::string>("controller_id", "FollowPath");
    declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    declare_parameter<std::string>("progress_checker_id", "progress_checker");

    get_parameter("agent_num", agent_num_);
    get_parameter("global_plan_topic", global_plan_topic_);
    get_parameter("global_frame_id", global_frame_id_);
    get_parameter("cancel_previous_goal_on_new_plan", cancel_previous_goal_on_new_plan_);
    get_parameter("executor_rate", executor_rate_);
    get_parameter("controller_id", controller_id_);
    get_parameter("goal_checker_id", goal_checker_id_);
    get_parameter("progress_checker_id", progress_checker_id_);

    agent_names_.resize(agent_num_);
    pending_plans_.resize(agent_num_);
    last_sent_plans_.resize(agent_num_);
    action_clients_.resize(agent_num_);
    active_goal_handles_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      declare_parameter<std::string>(
        "agent_name.agent_" + std::to_string(i),
        "carter" + std::to_string(i + 1));

      get_parameter("agent_name.agent_" + std::to_string(i), agent_names_[i]);

      const std::string action_name = "/" + agent_names_[i] + "/follow_path";
      action_clients_[i] = rclcpp_action::create_client<FollowPath>(this, action_name);

      RCLCPP_INFO(
        get_logger(),
        "Agent %d follow_path action client: %s",
        i, action_name.c_str());
    }

    sub_global_plan_ = create_subscription<mapf_msgs::msg::GlobalPlan>(
      global_plan_topic_, 10, std::bind(&PathFollowerExecutor::planCallback, this, _1));

    exec_thread_ = std::make_unique<std::thread>(&PathFollowerExecutor::execThread, this);

    RCLCPP_INFO(get_logger(), "PathFollowerExecutor initialized.");
  }

  ~PathFollowerExecutor() override
  {
    stop_requested_ = true;

    if (exec_thread_ && exec_thread_->joinable()) {
      exec_thread_->join();
    }
  }

private:
  int agent_num_;
  int make_span_;

  std::string global_plan_topic_;
  std::string global_frame_id_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string progress_checker_id_;

  double executor_rate_;

  bool cancel_previous_goal_on_new_plan_;
  bool has_new_plan_;
  bool stop_requested_;

  std::mutex plan_mtx_;

  std::vector<std::string> agent_names_;
  std::vector<mapf_msgs::msg::SinglePlan> pending_plans_;
  std::vector<mapf_msgs::msg::SinglePlan> last_sent_plans_;

  rclcpp::Subscription<mapf_msgs::msg::GlobalPlan>::SharedPtr sub_global_plan_;
  std::unique_ptr<std::thread> exec_thread_;

  std::vector<FollowPathActionClient::SharedPtr> action_clients_;
  std::vector<GoalHandleFollowPath::SharedPtr> active_goal_handles_;

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

  nav_msgs::msg::Path normalizePath(const mapf_msgs::msg::SinglePlan & single_plan)
  {
    nav_msgs::msg::Path out = single_plan.plan;
    out.header.frame_id = global_frame_id_;
    out.header.stamp = now();

    for (auto & pose_stamped : out.poses) {
      pose_stamped.header.frame_id = global_frame_id_;
      pose_stamped.header.stamp = out.header.stamp;

      // orientation이 비어 있으면 기본값 보정
      const auto & q = pose_stamped.pose.orientation;
      const bool quat_is_zero =
        std::abs(q.x) < 1e-12 &&
        std::abs(q.y) < 1e-12 &&
        std::abs(q.z) < 1e-12 &&
        std::abs(q.w) < 1e-12;

      if (quat_is_zero) {
        pose_stamped.pose.orientation.w = 1.0;
      }
    }

    return out;
  }

  FollowPath::Goal toFollowPathGoal(const nav_msgs::msg::Path & path)
  {
    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;
   // goal.progress_checker_id = progress_checker_id_; //jazzy (O) humble (X)
    return goal;
  }

  void cancelActiveGoal(int agent_idx)
  {
    if (!active_goal_handles_[agent_idx]) {
      return;
    }

    action_clients_[agent_idx]->async_cancel_goal(active_goal_handles_[agent_idx]);
    RCLCPP_INFO(get_logger(), "Agent %d cancel requested for previous FollowPath goal", agent_idx);
    active_goal_handles_[agent_idx].reset();
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
      if (msg->global_plan[i].plan.poses.empty()) {
        RCLCPP_WARN(get_logger(), "Agent %d received empty path. Skip.", i);
        continue;
      }

      if (!sameSinglePlan(last_sent_plans_[i], msg->global_plan[i])) {
        pending_plans_[i] = msg->global_plan[i];
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
    RCLCPP_INFO(get_logger(), "path_follower_thread: waiting for MAPF plans...");
    rclcpp::Rate loop_rate(executor_rate_);

    while (rclcpp::ok() && !stop_requested_) {
      loop_rate.sleep();

      if (!has_new_plan_) {
        continue;
      }

      std::unique_lock<std::mutex> lock(plan_mtx_);
      has_new_plan_ = false;
      auto local_plans = pending_plans_;
      lock.unlock();

      for (int i = 0; i < agent_num_; ++i) {
        if (local_plans[i].plan.poses.empty()) {
          continue;
        }

        if (!action_clients_[i]->wait_for_action_server(std::chrono::seconds(2))) {
          RCLCPP_ERROR(get_logger(), "Agent %d follow_path action server not available", i);
          continue;
        }

        if (cancel_previous_goal_on_new_plan_) {
          cancelActiveGoal(i);
        }

        nav_msgs::msg::Path path = normalizePath(local_plans[i]);
        auto goal_msg = toFollowPathGoal(path);

        RCLCPP_INFO(
          get_logger(),
          "Agent %d send FollowPath: poses=%zu frame=%s controller=%s",
          i,
          path.poses.size(),
          path.header.frame_id.c_str(),
          controller_id_.c_str());

        auto send_goal_options =
          rclcpp_action::Client<FollowPath>::SendGoalOptions();

        send_goal_options.goal_response_callback =
          [this, i](GoalHandleFollowPath::SharedPtr goal_handle)
          {
            if (!goal_handle) {
              RCLCPP_WARN(this->get_logger(), "Agent %d FollowPath goal rejected", i);
            } else {
              active_goal_handles_[i] = goal_handle;
              RCLCPP_INFO(this->get_logger(), "Agent %d FollowPath goal accepted", i);
            }
          };

        send_goal_options.feedback_callback =
          [this, i](
            GoalHandleFollowPath::SharedPtr,
            const std::shared_ptr<const FollowPath::Feedback> feedback)
          {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Agent %d FollowPath feedback: dist_to_goal=%.3f speed=%.3f",
              i, feedback->distance_to_goal, feedback->speed);
          };

        send_goal_options.result_callback =
          [this, i](const GoalHandleFollowPath::WrappedResult & result)
          {
            switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Agent %d FollowPath succeeded", i);
                break;
              case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(
                  this->get_logger(),
                  "Agent %d FollowPath aborted: error_code=%u msg=%s", i);
                //  i, result.result->error_code, result.result->error_msg.c_str()); //jazzy (O) humble (X)
                break;
              case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Agent %d FollowPath canceled", i);
                break;
              default:
                RCLCPP_WARN(this->get_logger(), "Agent %d FollowPath unknown result", i);
                break;
            }
          };

        action_clients_[i]->async_send_goal(goal_msg, send_goal_options);
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollowerExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
