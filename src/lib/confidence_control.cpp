#include "prox/confidence_control.h"

namespace prox { 

confidence_control::confidence_control(void) {
    this->nh_ = ros::NodeHandle("~confidence_control");

    this->current_position = nav_msgs::Odometry();
    this->current_goal = geometry_msgs::PoseStamped();

    ros::param::param("~short_distance", this->short_distance, 0.1);
    ros::param::param("~long_distance",  this->long_distance,  0.4);

    std::string odom = "odom";
    std::string goal = "goal";

    ros::param::param("~odom",  this->odom_topic_, odom);
    ros::param::param("~goal",  this->goal_topic_, goal);

    this->setup_listeners();
    this->setup_services();
}

confidence_control::~confidence_control(void) {}

bool confidence_control::setup_listeners() {
    this->odometry_sub_ = this->nh_.subscribe(this->odom_topic_, 1,
     &confidence_control::odom_callback, this);
    this->goal_sub_     = this->nh_.subscribe(this->goal_topic_, 1,
     &confidence_control::goal_callback, this);
    return true;
}

bool confidence_control::setup_services() {
    this->field_stard_srv = this->nh_.serviceClient<std_srvs::Empty>("/navigation/navigation_start");
    this->field_stop_srv  = this->nh_.serviceClient<std_srvs::Empty>("/navigation/navigation_stop");
    return true;
}

void confidence_control::run() {
    ros::Rate r(16);

    while(ros::ok()){
        this->check_control_state();

        if(! this->goal_reached_ && !this->running){
    this->field_stard_srv.call(this->empty);
            this->running = true;
        }else if(this->goal_reached_ && this->running){
            this->field_stop_srv.call(this->empty);
            this->running = false;

            //TODO: send the message to the planner that the goal is reached
        }

        ros::spinOnce();
        r.sleep();
    }
}

void confidence_control::check_control_state() {
    double d = std::pow((this->current_position.pose.pose.position.x - this->current_goal.pose.position.x),2) +
        std::pow((this->current_position.pose.pose.position.y - this->current_goal.pose.position.y),2);
    d = std::sqrt(d);

    if (d > this->short_distance) 
        this->goal_reached_ = false;
    if (d < this->long_distance)
        this->goal_reached_ = true;
}

void confidence_control::odom_callback(const nav_msgs::Odometry msg) {
    this->current_position = msg;
}
void confidence_control::goal_callback(const move_base_msgs::MoveBaseActionGoal msg) {
    this->current_goal = msg.goal.target_pose;
}

}
