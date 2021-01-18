/**
 * Copyright 2019 PAL Robotics S.L.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DDYNAMIC_RECONFIGURE_
#define _DDYNAMIC_RECONFIGURE_

#include <dynamic_reconfigure/server.h>
#include <ddynamic_reconfigure/registered_param.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure_utils.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <atomic>

namespace ddynamic_reconfigure
{
/**
 * @brief The DDynamicReconfigure class allows to use ROS dynamic reconfigure without the
 * need to write
 * a custom cpf file, variables are registered and exposed at run time.
 * Modification of the variables is done through a variable pointer or through a callback
 * function.
 */
class DDynamicReconfigure
{
public:
  /**
    * @param nh the queue associated to this nh should spined() somewhere else
    * @param auto_update - Update the variable values immediately on change by
    * service call. When it is true, it is not Thread Safe. In case it is set to
    * False, updateRegisteredVariablesData method needs to be called to update the
    * registered variables
    */
  DDynamicReconfigure(const ros::NodeHandle &nh = ros::NodeHandle("~"), bool auto_update = true);

  virtual ~DDynamicReconfigure();

  /**
   * @brief registerVariable register a variable to be modified via the
   * dynamic_reconfigure API. When a change is made, it will be reflected in the
   * variable directly
   * @deprecated In the future this method will be merged with the registerVariable
   * that takes a pointer and a callback, but with the callback being optional
   */
  template <typename T>
  void registerVariable(const std::string &name, T *variable,
                        const std::string &description = "", T min = getMin<T>(),
                        T max = getMax<T>(), const std::string &group = "Default");

  template <typename T>
  void registerEnumVariable(const std::string &name, T *variable,
                            const std::string &description = "",
                            std::map<std::string, T> enum_dict = {},
                            const std::string &enum_description = "",
                            const std::string &group = "Default");
  /**
   * @brief registerVariable like the functions above, but with a callback to be called
   * when the
   * variable is changed from the dynamic_reconfigure API.
   */
  template <typename T>
  void registerVariable(const std::string &name, T *variable,
                        const boost::function<void(T value)> &callback,
                        const std::string &description = "", T min = getMin<T>(),
                        T max = getMax<T>(), const std::string &group = "Default");

  template <typename T>
  void registerEnumVariable(const std::string &name, T *variable,
                            const boost::function<void(T)> &callback,
                            const std::string &description,
                            std::map<std::string, T> enum_dict = {},
                            const std::string &enum_description = "",
                            const std::string &group = "Default");

  /**
   * @brief registerVariable register a variable to be modified via the
   * dynamic_reconfigure API. When a change is made, the callback will be called with the
   * new value. The variable itself is not modified when registered using this method.
   * This method is useful for "dynamic reconfigure variables" that don't have a
   * direct equivalent in the C++ code, such as vector.size().
   */
  template <typename T>
  void registerVariable(const std::string &name, T current_value,
                        const boost::function<void(T value)> &callback,
                        const std::string &description = "", T min = getMin<T>(),
                        T max = getMax<T>(), const std::string &group = "Default");

  template <typename T>
  void registerEnumVariable(const std::string &name, T current_value,
                            const boost::function<void(T)> &callback,
                            const std::string &description,
                            std::map<std::string, T> enum_dict = {},
                            const std::string &enum_description = "",
                            const std::string &group = "Default");

  /**
   * @brief publishServicesTopics starts the server once all the needed variables are
   * registered
   */
  virtual void publishServicesTopics();

  virtual void updatePublishedInformation();

  typedef boost::function<void()> UserCallbackType;

  /**
   * @brief setUserCallback An optional callback that will be called whenever a value is
   * changed
   */
  virtual void setUserCallback(const UserCallbackType &callback);

  virtual void clearUserCallback();


  /**
   * Deprecated functions. For backwards compatibility, cannot be a template for legacy
   * reasons
   */
  virtual void RegisterVariable(double *variable, std::string id, double min = -100,
                                double max = 100);

  virtual void RegisterVariable(int *variable, std::string id, int min = -100, int max = 100);

  virtual void RegisterVariable(bool *variable, std::string id);

  virtual void PublishServicesTopics();

  /**
   * @brief updateRegisteredVariablesData - Method to be called to update the registered
   * variable, incase the auto_update is not choosen
   */
  virtual void updateRegisteredVariablesData();

protected:
  template <typename T>
  std::vector<std::unique_ptr<RegisteredParam<T>>> &getRegisteredVector();

  virtual dynamic_reconfigure::ConfigDescription generateConfigDescription() const;

  virtual dynamic_reconfigure::Config generateConfig();

  virtual bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                                 dynamic_reconfigure::Reconfigure::Response &rsp);

  virtual void updateConfigData(const dynamic_reconfigure::Config &config);

  /**
   * @brief setUserCallback Set a function to be called when parameters have changed
   */

  ros::NodeHandle node_handle_;
  ros::ServiceServer set_service_;
  ros::Publisher update_pub_;
  ros::Publisher descr_pub_;

  bool advertised_;
  bool auto_update_;

  std::atomic_bool new_config_avail_;

  // Registered variables
  std::vector<std::unique_ptr<RegisteredParam<int>>> registered_int_;
  std::vector<std::unique_ptr<RegisteredParam<double>>> registered_double_;
  std::vector<std::unique_ptr<RegisteredParam<bool>>> registered_bool_;
  std::vector<std::unique_ptr<RegisteredParam<std::string>>> registered_string_;
  std::vector<std::string> config_groups_;

  UserCallbackType user_callback_;

  ros::Timer pub_config_timer_;
  dynamic_reconfigure::Config last_config_;
  dynamic_reconfigure::Config updated_config_;
};

typedef boost::shared_ptr<DDynamicReconfigure> DDynamicReconfigurePtr;
}

#endif
