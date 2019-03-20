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
 * a custom cpf file, variables are registered and exposed at run time. 
 * Modification of the variables is done through a variable pointer or through a callback function.
 */
class DDynamicReconfigure
{
  template <typename T>
  class RegisteredParam
  {
  public:
    RegisteredParam(const std::string &name, const std::string &description,
                    T min_value, T max_value)
      : name_(name), description_(description), min_value_(min_value), max_value_(max_value)
    {
    }
    
    virtual T getCurrentValue() const = 0;
    virtual void updateValue(T new_value) = 0;
    
    
    const std::string name_;
    const std::string description_;
    const T min_value_;
    const T max_value_;
  };
  
  template <typename T>
  class PointerRegisteredParam : public RegisteredParam<T>
  {
  public:
    PointerRegisteredParam(const std::string &name, const std::string &description,
                            T min_value, T max_value, T *variable)
      : RegisteredParam<T>(name, description, min_value, max_value), variable_(variable)
    {      
    }
    
    virtual T getCurrentValue() const override
    {
      return *variable_;
    }
    virtual void updateValue(T new_value) override
    {
      *variable_ = new_value;
    }
    
  protected:
    T *variable_;
  };
  
  template <typename T>
  class CallbackRegisteredParam : public RegisteredParam<T>
  {
  public:
    CallbackRegisteredParam(const std::string &name, const std::string &description,
                            T min_value, T max_value, T current_value,
                            boost::function<void(T value)> callback)
      : RegisteredParam<T>(name, description, min_value, max_value)
      , current_value_(current_value)
      , callback_(callback)
    {      
    }
    
    virtual T getCurrentValue() const override
    {
      return current_value_;
    }
    virtual void updateValue(T new_value) override
    {
      callback_(new_value);
      current_value_ = new_value;
    }
    
    T current_value_;
    boost::function<void(T value)> callback_;
  };

public:
  /**
    * @param nh the queue associated to this nh should spined() somewhere else
    */
  DDynamicReconfigure(const ros::NodeHandle &nh = ros::NodeHandle("~"));

  virtual ~DDynamicReconfigure();

  /**
   * @brief registerVariable register a variable to be modified via the
   * dynamic_reconfigure API. When a change is made, it will be reflected in the
   * variable directly
   */
  void registerVariable(const std::string &name, int *variable,
                        const std::string &description = "", int min = -100, int max = 100);

  void registerVariable(const std::string &name, double *variable,
                        const std::string &description = "", double min = -100,
                        double max = 100);

  void registerVariable(const std::string &name, bool *variable,
                        const std::string &description = "");

  /**
   * @brief registerVariable register a variable to be modified via the
   * dynamic_reconfigure API. When a change is made, the callback will be called with the
   * new value
   */
  void registerVariable(const std::string &name, int current_value,
                        const boost::function<void(int value)> &callback,
                        const std::string &description = "", int min = -100, int max = 100);
  void registerVariable(const std::string &name, double current_value,
                        const boost::function<void(double value)> &callback,
                        const std::string &description = "", double min = -100,
                        double max = 100);
  void registerVariable(const std::string &name, bool current_value,
                        const boost::function<void(bool value)> &callback,
                        const std::string &description = "");

  /**
   * @brief publishServicesTopics starts the server once all the needed variables are
   * registered
   */
  void publishServicesTopics();

  void updatePublishedInformation();

  typedef boost::function<void()> UserCallbackType;
  
  /**
   * @brief setUserCallback An optional callback that will be called whenever a value is changed
   */
  void setUserCallback(const UserCallbackType &callback);

  void clearUserCallback();

  
  /**
   * Deprecated functions. For backwards compatibility
   */
  template <typename T>
  void RegisterVariable(T *variable, std::string id, double min = -100, double max = 100)
  {
    registerVariable(id, variable, "", min, max);
  }
  
  void RegisterVariable(bool *variable, std::string id)
  {
    registerVariable(id, variable, "");
  }
  
  void PublishServicesTopics();
private:
  dynamic_reconfigure::ConfigDescription generateConfigDescription() const;

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
  std::vector<std::unique_ptr<RegisteredParam<int>>> registered_int_;
  std::vector<std::unique_ptr<RegisteredParam<double>>> registered_double_;
  std::vector<std::unique_ptr<RegisteredParam<bool>>> registered_bool_;
  
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
