#include <memory>
#include <chrono>
#include <functional>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/nav.hpp"

using namespace std::chrono_literals;

class NavClient : public rclcpp::Node
{
public:
	using Nav = custom_interfaces::action::Nav;
	using GoalHandleNav = rclcpp_action::ClientGoalHandle<Nav>;

	NavClient()
	: Node("nav_client")
	{	my_callback_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);
		auto sub_options = rclcpp::SubscriptionOptions();
		sub_options.callback_group = my_callback_group_;
		
		client_ptr_ = rclcpp_action::create_client<Nav>(this, "nav_action",my_callback_group_);
		interface_timer_ = this->create_wall_timer(
        500ms, 
        std::bind(&NavClient::user_interface, this), 
        my_callback_group_
    );
	}

	void send_goal(float target)
	{
		if (!client_ptr_->wait_for_action_server(5s)) {
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
			return;
		}
		target_goal = target;
		auto goal_msg = Nav::Goal();
		goal_msg.goal = target;

		auto send_goal_options = rclcpp_action::Client<Nav>::SendGoalOptions();

		send_goal_options.goal_response_callback =std::bind(&NavClient::goalresponse_cb, this, std::placeholders::_1);
	
		send_goal_options.feedback_callback = std::bind(&NavClient::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);

		send_goal_options.result_callback = std::bind(&NavClient::result_cb, this, std::placeholders::_1);
			
		client_ptr_->async_send_goal(goal_msg, send_goal_options);
	}

private:
	rclcpp_action::Client<Nav>::SharedPtr client_ptr_;
	rclcpp::TimerBase::SharedPtr interface_timer_;
	GoalHandleNav::SharedPtr goal_handle_;
	rclcpp::CallbackGroup::SharedPtr my_callback_group_;
  	bool cancel_sent_{false};
	float target_goal;

	void goalresponse_cb(GoalHandleNav::SharedPtr goal_handle){
		if(!goal_handle)
			RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
		else
			RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
		goal_handle_ = goal_handle;
	};
	void feedback_cb(GoalHandleNav::SharedPtr goal_handle, const std::shared_ptr<const Nav::Feedback> feedback){
		float remaining = feedback->cur_pose - target_goal;
		//RCLCPP_INFO(this->get_logger(), "Feedback: cur_pose=%.3f", feedback->cur_pose);
		 if (cancel_sent_ || !goal_handle_) {
      		return;
		if (remaining < 1.0 && remaining > -1.0) {
		cancel_sent_ = true;
		//RCLCPP_WARN(this->get_logger(),"Remaining angle less than 1.0, cancelling goal...");
		client_ptr_->async_cancel_goal(goal_handle_);
		goal_handle_ = nullptr;
		}
    }
	}

	void result_cb(const GoalHandleNav::WrappedResult & result) {
		goal_handle_ = nullptr;
		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
				RCLCPP_INFO(this->get_logger(), "Result: done=%s", result.result->done ? "true" : "false");
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
				break;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_WARN(this->get_logger(), "Goal was canceled");
				break;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				break;
			}
	}

	void user_interface() {
		interface_timer_->cancel();
        while (rclcpp::ok()) {
            std::cout << "\n--- User Interface ---\n";
            std::cout << "1. Send Goal\n";
            std::cout << "2. Cancel Current Goal\n";
            std::cout << "Selection: ";
            
            int choice;
            std::cin >> choice;

            if (choice == 1) {
                float target;
                std::cout << "Enter target distance: ";
                std::cin >> target;
                this->send_goal(target);
            } else if (choice == 2) {
				if(!goal_handle_)
				(std::cout << "No active goal to cancel." << std::endl);
                else
				client_ptr_->async_cancel_goal(goal_handle_);
            }
        }
    }

};

int main(int argc, char ** argv)
{

	
/*
	rclcpp::init(argc, argv);
	auto node = std::make_shared<NavClient>();
	float target = 30.0f;
	if (argc > 1) {
		target = std::atof(argv[1]);
	}
	// send goal and spin until result is received
	node->send_goal(target);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
*/

	rclcpp::init(argc, argv);
	auto node = std::make_shared<NavClient>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	return 0;
}
