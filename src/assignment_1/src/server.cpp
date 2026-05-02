#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_interfaces/action/nav.hpp"
using namespace std::chrono_literals;

class Server : public rclcpp::Node
{
public:
	using Nav = custom_interfaces::action::Nav;
	using GoalHandleNav = rclcpp_action::ServerGoalHandle<Nav>;
	Server()
	: Node("Server")
	{
	//	my_callback_group_ = this->create_callback_group(
    //	rclcpp::CallbackGroupType::MutuallyExclusive);
		my_callback_group_ = this->create_callback_group(
			rclcpp::CallbackGroupType::Reentrant);
		auto sub_options = rclcpp::SubscriptionOptions();
		sub_options.callback_group = my_callback_group_;

		// publisher to /cmd_vel
		cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		// subscriber to /odom
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/odom", 10,std::bind(&Server::odom_callback, this, std::placeholders::_1),sub_options);
		// action server
		action_server_ = rclcpp_action::create_server<Nav>(
			this,
			"nav_action",
			std::bind(&Server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Server::handle_cancel, this, std::placeholders::_1),
			std::bind(&Server::handle_accepted, this, std::placeholders::_1),
			rcl_action_server_get_default_options(),
    		my_callback_group_);
		RCLCPP_INFO(this->get_logger(), "Server node started");
	}

private:
	rclcpp_action::Server<Nav>::SharedPtr action_server_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	std::mutex odom_mutex_;
	float current_pose_{0.0f};
	rclcpp::CallbackGroup::SharedPtr my_callback_group_;

	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
																					std::shared_ptr<const Nav::Goal> goal)
	{
		(void)uuid;
		RCLCPP_INFO(this->get_logger(), "Received goal request: %f", goal->goal);
		if (goal->goal < 0.0f) {
			RCLCPP_WARN(this->get_logger(), "Rejecting negative goal");
			return rclcpp_action::GoalResponse::REJECT;
		}
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav> goal_handle)
	{
		(void)goal_handle;
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
	{
		// run execution in a separate thread
		//std::thread{std::bind(&Server::execute, this, std::placeholders::_1), goal_handle}.detach();
		execute(goal_handle);
	}

	void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Executing goal...");
		rclcpp::Rate loop_rate(10);

		const auto goal = goal_handle->get_goal();
		auto feedback = std::make_shared<Nav::Feedback>();
		auto result = std::make_shared<Nav::Result>();

		// simple proportional controller: drive cmd_vel until current_pose reaches goal
		while (rclcpp::ok()) {
			if (goal_handle->is_canceling()) {
				// stop robot
				geometry_msgs::msg::Twist stop_msg;
				cmd_pub_->publish(stop_msg);
				result->done = false;
				goal_handle->canceled(result);
				RCLCPP_INFO(this->get_logger(), "Goal canceled");
				return;
			}

			float cur;
			{
				std::lock_guard<std::mutex> lk(odom_mutex_);
				cur = current_pose_;
			}

			feedback->cur_pose = cur;
			goal_handle->publish_feedback(feedback);

			float error = goal->goal - cur;
			if (std::fabs(error) < 0.05f) {
				// reached
				geometry_msgs::msg::Twist stop_msg;
				cmd_pub_->publish(stop_msg);
				result->done = true;
				goal_handle->succeed(result);
				RCLCPP_INFO(this->get_logger(), "Goal reached: %f", cur);
				return;
			}

			// publish velocity proportional to error (clamped)
			geometry_msgs::msg::Twist cmd;
			float v = error * 0.5f;
			if (v > 0.5f) v = 0.5f;
			if (v < -0.5f) v = -0.5f;
			cmd.linear.x = v;
			cmd_pub_->publish(cmd);

			loop_rate.sleep();
		}

		// if we exit loop unexpectedly
		result->done = false;
		goal_handle->abort(result);
	}

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
    std::lock_guard<std::mutex> lk(odom_mutex_);
    current_pose_ = msg->pose.pose.position.x;
    // Optional: Log it to check if it's working
    RCLCPP_INFO(this->get_logger(), "Current X: %f", current_pose_);
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Server>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	return 0;
}
