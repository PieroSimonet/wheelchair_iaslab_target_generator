#include "wheelchair_iaslab_target_generator/confidence_control.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "confidence_control");
    prox::confidence_control* cf = new prox::confidence_control();
    cf->run();
    return 0;
}
