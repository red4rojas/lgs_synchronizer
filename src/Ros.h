#ifndef NODE_H
#define NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include "lgs_interfaces/action/reel.hpp"
#include "lgs_interfaces/action/crawler.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("synchro_node"), __VA_ARGS__)

class Ros {
using CrawlerAction = lgs_interfaces::action::Crawler;
using CrawlGoalHandle = rclcpp_action::ClientGoalHandle<CrawlerAction>;
using ReelAction = lgs_interfaces::action::Reel;
using ReelGoalHandle = rclcpp_action::ClientGoalHandle<ReelAction>;
public:
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    static Ros *instance(void) { return s_self; }
    // void pass_command(const std::string& msg);
    static void quit(void) { s_self->shutdown(); }
    void spin(void);
    void spinOnBackground(void);
    void shutdown(void);
public:
    rclcpp::Node::SharedPtr node(void) { return m_node; }

private:
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);
    // Publishers, subscribers and servers
    rclcpp_action::Client<CrawlerAction>::SharedPtr m_crawl_client;
    rclcpp_action::Client<ReelAction>::SharedPtr m_reel_client;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_request_sub;
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    // Instance
    static Ros *s_self;
    rclcpp::Node::SharedPtr m_node;
private:
    void wait_servers();
    void call_crawler(signed short single);
    void call_crawler(std::vector<signed short> pattern); 
    void call_crawler(std::vector<signed short> pattern, std::vector<float> timing); 
    void call_reel(int vel, float interval, bool continous);
    void goal_response_callback(const CrawlGoalHandle::SharedPtr & goal_handle);
    void actuation_request_callback(const std_msgs::msg::String & msg);

    void reel_goal_response_callback(const ReelGoalHandle::SharedPtr & reel_goal_handle); 
    //  void feedback_callback(CrawlGoalHandle::SharedPtr,const std::shared_ptr<CrawlerAction::Feedback> feedback);
};

#endif // NODE_H
