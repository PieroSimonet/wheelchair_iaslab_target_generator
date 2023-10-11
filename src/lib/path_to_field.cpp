#include "prox/path_to_field.h"

namespace prox {

path_to_field::path_to_field() {
    this->initialize();
    this->setup_grid();
    this->setup_listeners();
    this->setup_services();
    this->setup_publishers();
}

path_to_field::~path_to_field() {}

void path_to_field::run() {
    ros::Rate rate(this->rate_);

    while(ros::ok()){
        this->segment_path();
        this->generate_attactor();
        this->attractor_pub_.publish(this->attractor_);
        ros::spinOnce();
        rate.sleep();
    }
}

void path_to_field::initialize() {
    this->nh_ = ros::NodeHandle("~");

    std::string frame_id   = "fused_scan";
    std::string base_frame = "base_footprint";
    std::string odom_frame = "odom";
  
    ros::param::param("~rate", this->rate_, 16);
    ros::param::param("~frame_id", this->frame_id_, frame_id);
    ros::param::param("~base_frame", this->base_frame, base_frame);
    ros::param::param("~odom_frame", this->odom_frame_, odom_frame);
    ros::param::param("~angle_min", this->angle_min_, -M_PI);
    ros::param::param("~angle_max", this->angle_max_,  M_PI);
    ros::param::param("~angle_increment", this->angle_increment_, M_PI/8);
    ros::param::param("~range_min", this->range_min_, 0.0d);
    ros::param::param("~range_max", this->range_max_, 5.0d);

    ros::param::param("~max_distance", this->max_distance_, 2.0d);
    ros::param::param("~delta_distance", this->delta_distance_, 0.8d);

    std::string odom_topic = "/odom";
    std::string path_topic = "/path";
    std::string goal_topic = "/move_base/goal";
    std::string attractor_topic = "/attractor";

    ros::param::param<std::string>("~odom_topic", this->odom_topic_, odom_topic);
    ros::param::param<std::string>("~path_topic", this->path_topic_, path_topic);
    ros::param::param<std::string>("~goal_topic", this->goal_topic_, goal_topic);
    ros::param::param<std::string>("~attractor_topic", this->attractor_topic_, attractor_topic);

    this->current_odom_ = nav_msgs::Odometry();
    this->current_goal_ = geometry_msgs::PoseStamped();
    this->attractor_ = proximity_grid::ProximityGridMsg();
    this->path_ = nav_msgs::Path();

    tf_listener = new tf2_ros::TransformListener(this->tf_buffer_);
}

void path_to_field::setup_grid() {
    this->attractor_.header.frame_id = this->frame_id_;
    this->attractor_.header.stamp = ros::Time::now();
    this->attractor_.header.seq = 0;

    this->attractor_.angle_min = this->angle_min_;
    this->attractor_.angle_max = this->angle_max_;
    this->attractor_.angle_increment = this->angle_increment_;
    this->attractor_.range_min = this->range_min_;
    this->attractor_.range_max = this->range_max_;    
}

void path_to_field::setup_listeners() {
    // TODO recheck these
    this->odom_sub_ = this->nh_.subscribe(this->odom_topic_, 1, &path_to_field::odom_callback, this);
    this->path_sub_ = this->nh_.subscribe(this->path_topic_, 1, &path_to_field::path_callback, this);
    this->goal_sub_ = this->nh_.subscribe(this->goal_topic_, 1, &path_to_field::goal_callback, this);
}

void path_to_field::setup_services() {}

void path_to_field::setup_publishers() {
    this->attractor_pub_ = this->nh_.advertise<proximity_grid::ProximityGridMsg>(this->attractor_topic_, 1);
}

void path_to_field::segment_path() {
    bool found = false;

    double current_distance = 0;

    for (int i = 0; i < this->path_.poses.size(); i++){
        if (!found){
            current_distance = sqrt(
              pow(this->current_odom_.pose.pose.position.x - this->path_.poses[i].pose.position.x, 2) +
              pow(this->current_odom_.pose.pose.position.y - this->path_.poses[i].pose.position.y, 2)
            );
            if (current_distance < this->max_distance_ && current_distance > this->delta_distance_){
                this->current_goal_.pose = this->path_.poses[i].pose;
                found = true;
            } 
        }
    }

    if (!found){
        this->current_goal_.pose = this->current_odom_.pose.pose;
        this->current_goal_.header.frame_id = this->odom_frame_;
    }
  
}

void path_to_field::generate_attactor() {

    geometry_msgs::TransformStamped transform;

    float distance = 0.0;
    double alpha   = 0.0;

    try{

        transform = this->tf_buffer_.lookupTransform(this->odom_frame_, this->base_frame, ros::Time(0), ros::Duration(1/this->rate_));

        geometry_msgs::Pose destination;
        tf2::doTransform(this->current_goal_.pose, destination, transform);

        distance = std::sqrt(std::pow(destination.position.x,2) 
                           + std::pow(destination.position.y,2));

        alpha = std::atan2(destination.position.y, destination.position.x);

    }catch (tf2::TransformException &ex) {

        ROS_WARN("Could NOT transform %s to %s: %s \n Set an attractor on itself", this->odom_frame_.c_str(), this->base_frame.c_str() ,ex.what());
        // DO NOT SET NEW DISTANCE -> Attractor will be in (0,0)
    }

    if (alpha < this->angle_min_)
      alpha = this->angle_min_;
    else if (alpha > this->angle_max_)
      alpha = this->angle_max_;

    bool used = false;

    int size = (this->angle_max_ - this->angle_min_) / this->angle_increment_;
    std::vector<float> attactors(size);

    for (int i = 0; i < size; i++){
        if (!used){
            if (alpha >= this->angle_min_ + i * this->angle_increment_ && alpha < this->angle_min_ + (i + 1) * this->angle_increment_){
                attactors[i] = distance;
                used = true;
            } else 
                attactors[i] = INFINITY;
        } else
            attactors[i] = INFINITY;
    }

    if (!used)
        attactors[size-1] = distance;

    this->attractor_.ranges = attactors;

}

void path_to_field::odom_callback(const nav_msgs::Odometry msg) {
    this->current_odom_ = msg;
}

void path_to_field::path_callback(const nav_msgs::Path msg) {
    this->path_ = msg;
    this->current_goal_ = msg.poses.back();
}

void path_to_field::goal_callback(const move_base_msgs::MoveBaseActionGoal msg) {
    this->current_goal_ = msg.goal.target_pose;
}

} // namespace prox
