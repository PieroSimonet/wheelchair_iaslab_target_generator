#include "wheelchair_iaslab_target_generator/target_generator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_generator");
    prox::target_generator* tg = new prox::target_generator();
    tg->run();
    return 0;
}
