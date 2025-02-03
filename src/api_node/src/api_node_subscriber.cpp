#include <drogon/drogon.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <thread>

class RosSubscriberHandler {
public:
    RosSubscriberHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        // Get configurable parameters (or use defaults)
        std::string topic_name = node_->declare_parameter<std::string>("topic_name", "api_topic");
        int queue_size = node_->declare_parameter<int>("queue_size", 10);

        // Create the subscriber
        subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            topic_name,
            queue_size,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_message_ = msg->data;
                RCLCPP_INFO(node_->get_logger(), "Received message: %s", last_message_.c_str());
            }
        );

        RCLCPP_INFO(node_->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
    }

    // HTTP endpoint handler to return the last received message
    void getLatestMessage(const drogon::HttpRequestPtr &req,
                          std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
        auto resp = drogon::HttpResponse::newHttpResponse();
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_message_.empty()) {
            resp->setStatusCode(drogon::k404NotFound);
            resp->setBody("No messages received yet.");
        } else {
            resp->setStatusCode(drogon::k200OK);
            resp->setBody("Latest received message: " + last_message_);
        }
        callback(resp);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::string last_message_;
    std::mutex mutex_;  // Protects access to last_message_
};

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ros_subscriber_node");

    // Start ROS2 spinning in a separate thread
    std::thread ros_spin_thread([node]() {
        rclcpp::spin(node);
        rclcpp::shutdown();
    });

    // Create an instance of the subscriber handler
    auto handler = std::make_shared<RosSubscriberHandler>(node);

    // Register an HTTP endpoint to retrieve the latest message
    drogon::app().registerHandler(
        "/latest_message",
        [handler](const drogon::HttpRequestPtr &req, 
                  std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            handler->getLatestMessage(req, std::move(callback));
        },
        {drogon::Get}
    );

    // Start the Drogon HTTP server on port 8080
    drogon::app().addListener("0.0.0.0", 8081).run();

    ros_spin_thread.join();
    return 0;
}
