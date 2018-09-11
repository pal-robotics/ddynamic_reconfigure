#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

using namespace ddynamic_reconfigure;

/**
  Topics:
  * /dynamic_tutorials/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
  * /dynamic_tutorials/parameter_updates [dynamic_reconfigure/Config]

  Services:
  * /dynamic_tutorials/set_parameter:  dynamic_reconfigure/Reconfigure
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_dynamic_reconfigure");

  ros::NodeHandle nh("fake_dyanmic_reconfigure");

  double double_test = 0.0;
  double double_range = 2;
  int int_test = 0;
  bool bool_test = false;
  double changing_variable = 0.0;
  DDynamicReconfigure ddr(nh);
  DDynamicReconfigure ddr2(ros::NodeHandle(nh, "nh2"));
  DDynamicReconfigure ddr3(ros::NodeHandle(nh, "nh3"));

  ddr.RegisterVariable(&double_test, "double_test");
  ddr.RegisterVariable(&double_range, "double_range_test", 0, 10);
  ddr.RegisterVariable(&int_test, "int_test");
  ddr.RegisterVariable(&bool_test, "bool_test");
  ddr.RegisterVariable(&changing_variable, "changing_variable");

  ddr2.RegisterVariable(&double_test, "double_test");
  ddr2.RegisterVariable(&int_test, "int_test");
  ddr2.RegisterVariable(&bool_test, "bool_test");

  ddr.PublishServicesTopics();
  ddr2.PublishServicesTopics();
  ddr3.PublishServicesTopics();


  ROS_INFO("Spinning node");

  while (nh.ok())
  {
    changing_variable += 0.5;
    std::cerr << "changing_variable " << changing_variable << std::endl;
    std::cerr << "double " << double_test << std::endl;
    std::cerr << "double range" << double_range << std::endl;
    std::cerr << "int " << int_test << std::endl;
    std::cerr << "bool " << bool_test << std::endl;
    std::cerr << "*********" << std::endl;
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  return 0;
}
