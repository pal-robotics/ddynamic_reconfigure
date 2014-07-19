#include <ros/ros.h>
#include <ddynamic_reconfigure/DDynamicReconfigure.h>

/**
  Topics:
  * /dynamic_tutorials/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
  * /dynamic_tutorials/parameter_updates [dynamic_reconfigure/Config]

  Services:
  * /dynamic_tutorials/set_parameter:  dynamic_reconfigure/Reconfigure
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_dynamic_reconfigure");

  ros::NodeHandle nh;

  double double_test = 0.0;
  int int_test = 0;
  bool bool_test = false;

  DDynamicReconfigure ddr(nh);
  ddr.RegisterVariable(&double_test, "double_test");
  ddr.RegisterVariable(&int_test, "int_test");
  ddr.RegisterVariable(&bool_test, "bool_test");

  ddr.PublishServicesTopics();

  ROS_INFO("Spinning node");

  while(nh.ok()){

    std::cerr<<"double "<<double_test<<std::endl;
    std::cerr<<"int "<<int_test<<std::endl;
    std::cerr<<"bool "<<bool_test<<std::endl;

    ros::spinOnce();
    ros::Duration(1.0).sleep();

  }
  return 0;
}

