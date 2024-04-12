#include "wheelchair_iaslab_target_generator/target_generator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_generator");
    wheelchair_iaslab_target_generator::target_generator* tg = new wheelchair_iaslab_target_generator::target_generator();
    tg->run();
    return 0;
}
