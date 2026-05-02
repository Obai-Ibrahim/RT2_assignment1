#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_interfaces/action/nav.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace nav_system {
class Server : public rclcpp::Node
{
public:
	using Nav = custom_interfaces::action::Nav;
	using GoalHandleNav = rclcpp_action::ServerGoalHandle<Nav>;
	Server(const rclcpp::NodeOptions & options)
	: Node("Server", options)
	{// Inside NavServer constructor
	this->get_logger().set_level(rclcpp::Logger::Level::Warn);	
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
	float err_pose_{0.0f};
	float err_th_{0.0f};
	float curr_x_{0.0f};
	float curr_y_{0.0f};
	float robot_theta_{0.0f};
	int init = 1;
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
											std::shared_ptr<const Nav::Goal> )
	{
		(void)uuid;
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
				stop_msg.angular.z = 0;
				stop_msg.linear.x = 0;
				cmd_pub_->publish(stop_msg);
				result->done = false;
				goal_handle->canceled(result);
				RCLCPP_INFO(this->get_logger(), "Goal canceled");
				return;
			}

			{
				std::lock_guard<std::mutex> lk(odom_mutex_);
			}

			float ex = goal->x - curr_x_;
			float ey = goal->y - curr_y_;
			float eth = goal->theta - robot_theta_;
			err_pose_=sqrt(ex*ex+ey*ey);
			float angle_to_goal = std::atan2(ey, ex);
       		float steering_error = angle_to_goal - robot_theta_;
			err_th_ = eth;

			feedback->err_pose = err_pose_;
			feedback->err_th = err_th_;
			float w = 0.0f;
			goal_handle->publish_feedback(feedback);
			float scaleRotationRate = 0.50f;
			float orientation_error = 0.0f;
			if (err_pose_ > 0.05) {
				// PHASE 1: Heading to the position (using your atan2 logic)
				// This makes the robot face the "Center" of the goal frame
				while (steering_error> M_PI) steering_error -= 2.0 * M_PI;
            	while (steering_error < -M_PI) steering_error += 2.0 * M_PI;
				w= scaleRotationRate * steering_error;
			} else {
				// PHASE 2: Matching the goal's actual orientation (using theta)
				// This makes the robot align with the "Rotation" of the goal frame
				orientation_error = goal->theta - robot_theta_;
				
				// Normalize to keep it between -PI and PI
				while (orientation_error > M_PI) orientation_error -= 2.0 * M_PI;
				while (orientation_error < -M_PI) orientation_error += 2.0 * M_PI;

				w = scaleRotationRate * orientation_error;
			}
			
			

			if (std::fabs(err_pose_) < 0.05f && std::fabs(err_th_) < 0.05f)
			{
				// reached
				geometry_msgs::msg::Twist stop_msg;
				stop_msg.angular.z = 0;
				stop_msg.linear.x = 0;
				cmd_pub_->publish(stop_msg);
				result->done = true;
				goal_handle->succeed(result);
				return;
			}
			// publish velocity proportional to error (clamped)
			geometry_msgs::msg::Twist cmd;
			float v = err_pose_ * 0.5f;
			if (v > 0.5f) v = 0.5f;
			if (v < -0.5f) v = -0.5f;
			if (w > 0.5f) w = 0.5f;
			if (w < -0.5f) w = -0.5f;
			if (std::fabs(err_pose_) < 0.05f)
				v = 0.0f;
			cmd.linear.x = v;
			cmd.angular.z = w;
			cmd_pub_->publish(cmd);
			loop_rate.sleep();
		}

		// if we exit loop unexpectedly
		result->done = false;
		goal_handle->abort(result);
	}

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		if (init==1){
		geometry_msgs::msg::Twist cmd1;
		cmd1.linear.x = 0; 
		cmd1.angular.z = 0;
		cmd_pub_->publish(cmd1);
		init = 2;
	}

    std::lock_guard<std::mutex> lk(odom_mutex_); 
    curr_x_ = msg->pose.pose.position.x;
    curr_y_ = msg->pose.pose.position.y;
	robot_theta_ = tf2::getYaw(msg->pose.pose.orientation);
	}
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(nav_system::Server)


