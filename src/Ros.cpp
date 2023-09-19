#include "Ros.h"
#include <thread>
#include <functional>
#include <future>
#include <signal.h>

using namespace std::placeholders;

void kill(int /*sig*/) {
}

Ros *Ros::s_self = nullptr;

Ros::Ros(int argc, char *argv[], const std::string &node_name) {
    // Initilize ROS
    rclcpp::init(argc, argv);
    // Create ROS executer and node
    m_executor = rclcpp::executors::StaticSingleThreadedExecutor::make_shared();
    m_node = rclcpp::Node::make_shared(node_name);
    m_executor->add_node(m_node);
    // Add ROS publisher and subscribers and action client/servers
    m_request_sub = m_node->create_subscription<std_msgs::msg::String>("lgs_actuation_requests", 10, 
    std::bind(&Ros::actuation_request_callback, this, _1));
    m_crawl_client = rclcpp_action::create_client<CrawlerAction>(m_node,"activate_crawler"); 
    m_reel_client = rclcpp_action::create_client<ReelAction>(m_node,"turn_reel"); 

    if (s_self) {
        LOG("Ops, only one instance of 'Ros' can be created!");
    }
    else {
        s_self = this; 
        LOG("Ros created...");
    }
    signal(SIGINT, kill);
}

Ros::~Ros() {
    rclcpp::shutdown(); 
    LOG("ROS shutdown!");
}

void Ros::spin(void) {
    m_executor->spin();
}

void Ros::spinOnBackground(void) {
    std::thread thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, m_executor));
    thread.detach();
}

void Ros::shutdown(void) {
    m_executor->cancel();
}

void Ros::call_crawler(signed short code){
    this->wait_servers();
    auto crawler_goal = CrawlerAction::Goal();
    crawler_goal.command.pattern = {code};
    crawler_goal.command.looping = false;
    auto send_goal_options = rclcpp_action::Client<CrawlerAction>::SendGoalOptions();
    // send_goal_options.goal_response_callback = std::bind(&Ros::goal_response_callback, this, _1);
    this->m_crawl_client->async_send_goal(crawler_goal, send_goal_options);
}

void Ros::call_crawler(std::vector<signed short> pattern){
    this->wait_servers();
    LOG("Calling crawler Server");
    auto crawler_goal = CrawlerAction::Goal();
    crawler_goal.command.pattern = rosidl_runtime_cpp::BoundedVector<int16_t, 6UL, std::allocator<int16_t>>(
      pattern.begin(), pattern.end()
    );
    if (pattern[0] == 0){
      crawler_goal.command.looping = false;
    } else {
      crawler_goal.command.looping = true;
    }
    auto send_goal_options = rclcpp_action::Client<CrawlerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Ros::goal_response_callback, this, _1);
    this->m_crawl_client->async_send_goal(crawler_goal, send_goal_options);
}

void Ros::call_crawler(std::vector<signed short> pattern, std::vector<float> timing){
    this->wait_servers();
    LOG("Calling crawler Server");
    auto crawler_goal = CrawlerAction::Goal();
    crawler_goal.command.pattern = rosidl_runtime_cpp::BoundedVector<int16_t, 6UL, std::allocator<int16_t>>(
      pattern.begin(), pattern.end()
    );
    crawler_goal.command.timing = rosidl_runtime_cpp::BoundedVector<float, 6UL, std::allocator<float>>(
      timing.begin(), timing.end()
    );
    if (pattern[0] == 0){
      crawler_goal.command.looping = false;
    } else {
      crawler_goal.command.looping = true;
    }
    auto send_goal_options = rclcpp_action::Client<CrawlerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Ros::goal_response_callback, this, _1);
    this->m_crawl_client->async_send_goal(crawler_goal, send_goal_options);
}

void Ros::call_reel(int vel, float interval, bool continuous){
    LOG("Calling reel server");
    auto reel_goal = ReelAction::Goal();
    reel_goal.command.velocity = vel;
    reel_goal.command.interval = interval;
    reel_goal.command.continuous = continuous;
    auto send_goal_options = rclcpp_action::Client<ReelAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Ros::reel_goal_response_callback, this, _1);
    this->m_reel_client->async_send_goal(reel_goal, send_goal_options);
}

void Ros::wait_servers(){
    if (!this->m_crawl_client->wait_for_action_server(timeout)) {
    LOG("Crawler server not available after waiting");
    // DO SOMETHING
  }

  if (!this->m_reel_client->wait_for_action_server(timeout)) {
    LOG("Reel server not available after waiting");
    // DO SOMETHING
  }
}

void Ros::goal_response_callback(const CrawlGoalHandle::SharedPtr & goal_handle)  {
  // auto goal_handle = future.get();
  if (!goal_handle) {
    LOG( "Goal was rejected by server");
  } else {
    LOG( "Goal accepted by server, waiting for result");
  }
}

void Ros::reel_goal_response_callback(const ReelGoalHandle::SharedPtr & reel_goal_handle)  {
  // auto goal_handle = future.get();
  if (!reel_goal_handle) {
    LOG( "Goal was rejected by server");
  } else {
    LOG( "Goal accepted by server, waiting for result");
  }
}


void Ros::actuation_request_callback(const std_msgs::msg::String & ros_msg){
  // pass_command(msg.data.c_str());
  auto msg = ros_msg.data;
  LOG( "received command: %s", msg.c_str());
  if (msg == "front_grip_on"){
    call_crawler(1);
  } else if (msg == "front_grip_off"){
    call_crawler(2); 
  } else if (msg == "extender_on"){
    call_crawler(3);
  } else if (msg == "extender_off"){
    call_crawler(4);
  } else if (msg == "back_grip_on"){
    call_crawler(5);
  } else if (msg == "back_grip_off"){
    call_crawler(6);   
  } else if (msg == "forward"){
    call_crawler(std::vector<signed short>({5,3,1,6,4,2}), std::vector<float>({5,7.0,0.8,0.8,7.5,0.8}));
    call_reel(-1,3.2,true);  
  // } else if (msg == "&forward"){
  //   call_crawler(std::vector<signed short>({5,3,1,6,4,2}), std::vector<float>({5,7.0,0.8,0.8,7,0.8}));
  //   call_reel(-1,3.2,true);    
  } else if (msg == "backward"){
    call_crawler(std::vector<signed short>({1,3,5,2,4,6}), std::vector<float>({5,7.0,0.8,0.8,7,0.8}));
    // call_crawler(std::vector<signed short>({2,4,6}));
    call_reel(2,0.8,true);   
  } else if (msg == "pull_tether"){
    call_crawler(std::vector<signed short>({2,4,6}));
    call_reel(2,0.8,true);
  } else if (msg == "unwind_tether"){
    call_reel(-1,0.5,true);     
  } else if (msg == "stop"){
    call_crawler(std::vector<signed short>({0}));
    call_reel(0,0,false);   
  } else if (msg == "stop_reel"){
    call_reel(0,0,false);     
  } else if (msg == "release_all"){
    call_crawler(std::vector<signed short>({2,4,6}));
  } else {
    LOG("Ops, invalid msg");
  }
}