///////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2014, 2015 PAL Robotics S.L.

// All rights reserved.

//////////////////////////////////////////////////////////////////////////////

// Author Hilario Tomé

#ifndef _DDYNAMIC_RECONFIGURE_
#define _DDYNAMIC_RECONFIGURE_

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

namespace ddynamic_reconfigure
{
/**
 * @brief The DDynamicReconfigure class allows to use ROS dynamic reconfigure without the
 * need to write
 * a custom cpf file, variables are register and exposed at run time
 */
class DDynamicReconfigure
{
  struct RegisteredInt
  {
    std::string name;
    int *value;
    int max_value;
    int min_value;

    RegisteredInt()
    {
      max_value = 100;
      min_value = -100;
    }
  };

  struct RegisteredDouble
  {
    std::string name;
    double *value;
    double max_value;
    double min_value;

    RegisteredDouble()
    {
      max_value = 100;
      min_value = -100;
    }
  };

public:
  /**
   * @param spin_thread will spin a ROS thread for publishing updates to the parameters.
   * If false it will use the global callback queue.
   * 
   * If you have a global ros::spin(), it's better to set it to false.
   */
  DDynamicReconfigure(const ros::NodeHandle &nh = ros::NodeHandle("~"));

  virtual ~DDynamicReconfigure();

  void RegisterVariable(int *variable, std::string id);

  void RegisterVariable(int *variable, std::string id, double min, double max);

  void RegisterVariable(double *variable, std::string id);

  void RegisterVariable(double *variable, std::string id, double min, double max);

  void RegisterVariable(bool *variable, std::string id);

  /**
   * @brief PublishServicesTopics stars the server once all the need variables are
   * registered
   */
  void PublishServicesTopics();

  void updatePublishedInformation();

  typedef boost::function<void()> UserCallbackType;
  void setUserCallback(const UserCallbackType &callback);

  void clearUserCallback();

private:
  void generateConfigDescription();

  dynamic_reconfigure::Config generateConfig();

  bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                         dynamic_reconfigure::Reconfigure::Response &rsp);

  /**
   * @brief setUserCallback Set a function to be called when parameters have changed
   */

  ros::NodeHandle node_handle_;
  ros::ServiceServer set_service_;
  ros::Publisher update_pub_;
  ros::Publisher descr_pub_;

  bool advertized_;

  // Registered variables
  std::vector<RegisteredInt> registered_int_;
  std::vector<RegisteredDouble> registered_double_;
  std::vector<std::pair<std::string, bool *> > registered_bool_;

  dynamic_reconfigure::ConfigDescription configDescription_;
  
  UserCallbackType user_callback_;
  
  ros::Timer pub_config_timer_;
  dynamic_reconfigure::Config last_config_;
};

typedef boost::shared_ptr<DDynamicReconfigure> DDynamicReconfigurePtr;

//// Hack until this is moved to pal namespace
// namespace pal
//{
// typedef ::ddynamic_reconfigure::DDynamicReconfigure DDynamicReconfigure;
//}
}

#endif
