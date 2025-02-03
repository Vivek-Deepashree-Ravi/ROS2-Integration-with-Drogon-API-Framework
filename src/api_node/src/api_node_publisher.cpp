#include <drogon/drogon.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RosHandler {
public:
    RosHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        // Allow topic name to be configurable via ROS2 parameters
        std::string topic_name = node_->declare_parameter<std::string>("topic_name", "api_topic");
        int queue_size = node_->declare_parameter<int>("queue_size", 10);

        publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, queue_size);

        RCLCPP_INFO(node_->get_logger(), "Publishing to topic: %s with queue size: %d", topic_name.c_str(), queue_size);
    }

    void publishMessage(const drogon::HttpRequestPtr &req, std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
        auto json = req->getJsonObject();
        auto resp = drogon::HttpResponse::newHttpResponse();
    
        // Use isMember() instead of contains()
        if (!json || !json->isMember("message")) {
            resp->setStatusCode(drogon::k400BadRequest);
            resp->setBody("Error: Invalid JSON, expected {\"message\": \"your text\"}");
            callback(resp);
            return;
        }
    
        std::string message = (*json)["message"].asString();
    
        if (message.empty()) {
            resp->setStatusCode(drogon::k400BadRequest);
            resp->setBody("Error: Message cannot be empty");
            callback(resp);
            return;
        }
    
        // Publish to ROS2 topic
        auto msg = std_msgs::msg::String();
        msg.data = message;
        publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published message: %s", message.c_str());
    
        // Respond to HTTP client
        resp->setStatusCode(drogon::k200OK);
        resp->setBody("Message published: " + message);
        callback(resp);
    }


private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("drogon_ros_node");

    // Start ROS2 spinning in a separate thread
    auto ros_thread = std::thread([ros_node]() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(ros_node);
            rate.sleep();
        }
        rclcpp::shutdown();
    });

    // Create handler instance
    auto handler = std::make_shared<RosHandler>(ros_node);

    // Register JSON POST API for publishing messages
    drogon::app().registerHandler(
        "/publish",
        [handler](const drogon::HttpRequestPtr &req, std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            handler->publishMessage(req, std::move(callback));
        },
        {drogon::Post} // Only allow POST requests
    );

    // Start Drogon HTTP server
    drogon::app().addListener("0.0.0.0", 8080).run();

    ros_thread.join();
    return 0;
}
