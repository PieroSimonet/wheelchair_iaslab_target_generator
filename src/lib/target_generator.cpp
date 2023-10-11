#include "prox/target_generator.h"

namespace prox {

target_generator::target_generator() {
    this->initialize();
    this->setup_listeners();
    this->setup_publishers();
    this->setup_services();
}

target_generator::~target_generator() {}

void target_generator::run() {
    while (ros::ok()) {
        ros::spinOnce();
    } 
}

void target_generator::initialize() {
    this->nh_ = ros::NodeHandle("~");

    std::string bci_topic  = "/integrator/neuroprediction";
    std::string odom_topic = "/odom";

    ros::param::param("~rate",       this->rate_, 16);
    ros::param::param("~bci_topic",  this->bci_topic_,  bci_topic);
    ros::param::param("~odom_topic", this->odom_topic_, odom_topic);

    this->bci_state_ = rosneuro_msgs::NeuroOutput();

    tf_listener_ = new tf2_ros::TransformListener(this->tf_buffer_);
}

void target_generator::setup_listeners() {
    this->bci_sub_ = this->nh_.subscribe(this->bci_topic_, 1, &target_generator::bci_callback, this);
}
void target_generator::setup_publishers() {}
void target_generator::setup_services() {}


void target_generator::bci_callback(const rosneuro_msgs::NeuroOutput msg) {
    this->bci_state_ = msg;
}




} // namespace prox
