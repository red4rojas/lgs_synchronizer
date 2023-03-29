#ifndef NODE_H
#define NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include "pipecrawler/action/crawleraction.hpp"
#include "reel/action/reelaction.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("synchro_node"), __VA_ARGS__)

class Ros {
using CrawlerAction = pipecrawler::action::Crawleraction;
using CrawlGoalHandle = rclcpp_action::ClientGoalHandle<CrawlerAction>;
using ReelAction = reel::action::Reelaction;
using ReelGoalHandle = rclcpp_action::ClientGoalHandle<ReelAction>;
public:
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    static Ros *instance(void) { return s_self; }
    void pass_command(const std::string& msg);
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
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    // Instance
    static Ros *s_self;
    rclcpp::Node::SharedPtr m_node;
private:
    void wait_servers();
    void call_crawler(signed short single);
    void call_crawler(std::vector<signed short> pattern); 
    void call_reel(int vel, float interval, bool continous);
    void goal_response_callback(std::shared_future<CrawlGoalHandle::SharedPtr> future);

};

#endif // NODE_H
