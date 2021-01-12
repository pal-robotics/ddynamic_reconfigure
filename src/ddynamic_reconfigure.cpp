#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <boost/make_unique.hpp>
namespace ddynamic_reconfigure
{
DDynamicReconfigure::DDynamicReconfigure(const ros::NodeHandle &nh, bool auto_update)
  : node_handle_(nh), advertised_(false), auto_update_(auto_update), new_config_avail_(false)
{
  pub_config_timer_ =
      nh.createTimer(ros::Duration(5.0),
                     boost::bind(&DDynamicReconfigure::updatePublishedInformation, this));
}

DDynamicReconfigure::~DDynamicReconfigure()
{
  set_service_.shutdown();
  update_pub_.shutdown();
  descr_pub_.shutdown();
}

void DDynamicReconfigure::publishServicesTopics()
{
  descr_pub_ = node_handle_.advertise<dynamic_reconfigure::ConfigDescription>(
      "parameter_descriptions", 1, true);
  const dynamic_reconfigure::ConfigDescription config_description = generateConfigDescription();
  descr_pub_.publish(config_description);

  config_groups_.clear();
  for (const auto &g : config_description.groups)
  {
    config_groups_.push_back(g.name);
  }

  update_pub_ =
      node_handle_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

  update_pub_.publish(generateConfig());

  set_service_ = node_handle_.advertiseService("set_parameters",
                                               &DDynamicReconfigure::setConfigCallback, this);

  advertised_ = true;
}

template <>
std::vector<std::unique_ptr<RegisteredParam<int>>> &DDynamicReconfigure::getRegisteredVector()
{
  return registered_int_;
}

template <>
std::vector<std::unique_ptr<RegisteredParam<double>>> &DDynamicReconfigure::getRegisteredVector()
{
  return registered_double_;
}

template <>
std::vector<std::unique_ptr<RegisteredParam<bool>>> &DDynamicReconfigure::getRegisteredVector()
{
  return registered_bool_;
}

template <>
std::vector<std::unique_ptr<RegisteredParam<std::string>>> &DDynamicReconfigure::getRegisteredVector()
{
  return registered_string_;
}


template <typename T>
void DDynamicReconfigure::registerVariable(const std::string &name, T *variable,
                                           const std::string &description, T min, T max,
                                           const std::string &group)
{
  registerVariable(name, variable, {}, description, min, max, group);
}

template <typename T>
void DDynamicReconfigure::registerEnumVariable(const std::string &name, T *variable,
                                               const std::string &description,
                                               std::map<std::string, T> enum_dict,
                                               const std::string &enum_description,
                                               const std::string &group)
{
  registerEnumVariable(name, variable, {}, description, enum_dict, enum_description, group);
}

template <typename T>
void DDynamicReconfigure::registerVariable(const std::string &name, T *variable,
                                           const boost::function<void(T value)> &callback,
                                           const std::string &description, T min, T max,
                                           const std::string &group)
{
  attemptGetParam(node_handle_, name, *variable, *variable);
  getRegisteredVector<T>().push_back(boost::make_unique<PointerRegisteredParam<T>>(
      name, description, min, max, variable, callback, std::map<std::string, T>(), "", group));
}

template <typename T>
void DDynamicReconfigure::registerEnumVariable(const std::string &name, T *variable,
                                               const boost::function<void(T value)> &callback,
                                               const std::string &description,
                                               std::map<std::string, T> enum_dict,
                                               const std::string &enum_description,
                                               const std::string &group)
{
  T min, max;
  std::tie(min, max) = getMinMax(enum_dict);
  attemptGetParam(node_handle_, name, *variable, *variable);
  getRegisteredVector<T>().push_back(boost::make_unique<PointerRegisteredParam<T>>(
      name, description, min, max, variable, callback, enum_dict, enum_description, group));
}

template <typename T>
void DDynamicReconfigure::registerVariable(const std::string &name, T current_value,
                                           const boost::function<void(T value)> &callback,
                                           const std::string &description, T min, T max,
                                           const std::string &group)
{
  attemptGetParam(node_handle_, name, current_value, current_value);
  getRegisteredVector<T>().push_back(boost::make_unique<CallbackRegisteredParam<T>>(
      name, description, min, max, current_value, callback, std::map<std::string, T>(), "", group));
}


template <typename T>
void DDynamicReconfigure::registerEnumVariable(const std::string &name, T current_value,
                                               const boost::function<void(T)> &callback,
                                               const std::string &description,
                                               std::map<std::string, T> enum_dict,
                                               const std::string &enum_description,
                                               const std::string &group)
{
  T min, max;
  std::tie(min, max) = getMinMax(enum_dict);
  attemptGetParam(node_handle_, name, current_value, current_value);
  getRegisteredVector<T>().push_back(boost::make_unique<CallbackRegisteredParam<T>>(
      name, description, min, max, current_value, callback, enum_dict, enum_description, group));
}

template <typename ParamType>
bool confCompare(const ParamType &a, const ParamType &b)
{
  return (a.name == b.name) && (a.value == b.value);
}

template <>
bool confCompare(const dynamic_reconfigure::DoubleParameter &a,
                 const dynamic_reconfigure::DoubleParameter &b)
{
  return (a.name == b.name) &&
         (std::fabs(a.value - b.value) < std::numeric_limits<double>::epsilon());
}

void DDynamicReconfigure::updatePublishedInformation()
{
  dynamic_reconfigure::Config config_msg = generateConfig();

  bool has_changed = false;
  has_changed = has_changed || config_msg.ints.size() != last_config_.ints.size();
  has_changed = has_changed || config_msg.doubles.size() != last_config_.doubles.size();
  has_changed = has_changed || config_msg.bools.size() != last_config_.bools.size();

  has_changed = has_changed ||
                !std::equal(config_msg.ints.begin(), config_msg.ints.end(),
                            last_config_.ints.begin(),
                            confCompare<dynamic_reconfigure::IntParameter>);
  has_changed = has_changed ||
                !std::equal(config_msg.doubles.begin(), config_msg.doubles.end(),
                            last_config_.doubles.begin(),
                            confCompare<dynamic_reconfigure::DoubleParameter>);
  has_changed = has_changed ||
                !std::equal(config_msg.bools.begin(), config_msg.bools.end(),
                            last_config_.bools.begin(),
                            confCompare<dynamic_reconfigure::BoolParameter>);

  if (has_changed)
  {
    last_config_ = config_msg;
    ROS_DEBUG_STREAM("Publishing ddr");
    update_pub_.publish(config_msg);
  }
}

bool DDynamicReconfigure::setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                                            dynamic_reconfigure::Reconfigure::Response &rsp)
{
  ROS_DEBUG_STREAM("Called config callback of ddynamic_reconfigure");

  updated_config_ = req.config;
  if (auto_update_)
  {
    updateConfigData(updated_config_);
  }
  else
  {
    new_config_avail_ = true;
    const ros::Time start_time = ros::Time::now();
    const ros::Duration timeout = ros::Duration(2.0);
    while ((ros::Time::now() - start_time) < timeout && new_config_avail_)
    {
      // Wait until the updateRegisteredVariablesData method is called by the user or
      // until the timeout
      ros::Duration(0.01).sleep();
    }
    if ((ros::Time::now() - start_time) > timeout)
    {
      ROS_ERROR_STREAM(
          "Timeout waiting to update the registered variable data! Registered Variable data update failed!");
      return true;
    }
  }

  /*
    boost::recursive_mutex::scoped_lock lock(mutex_);

    ConfigType new_config = config_;
    new_config.__fromMessage__(req.config);
    new_config.__clamp__();
    uint32_t level = config_.__level__(new_config);

    callCallback(new_config, level);

    updateConfigInternal(new_config);
    new_config.__toMessage__(rsp.config);
    */

  /*
      ConfigType new_config = config_;
      new_config.__fromMessage__(req.config);
      new_config.__clamp__();
      uint32_t level = config_.__level__(new_config);

      callCallback(new_config, level);

      updateConfigInternal(new_config);
      new_config.__toMessage__(rsp.config);
     */
  // std::cerr<<req.config<<std::endl;

  if (user_callback_)
  {
    try
    {
      user_callback_();
    }
    catch (std::exception &e)
    {
      ROS_WARN("Reconfigure callback failed with exception %s: ", e.what());
    }
    catch (...)
    {
      ROS_WARN("Reconfigure callback failed with unprintable exception.");
    }
  }

  dynamic_reconfigure::Config config_msg = generateConfig();
  update_pub_.publish(config_msg);
  rsp.config = config_msg;

  pub_config_timer_.setPeriod(ros::Duration(5.0));
  return true;
}

void DDynamicReconfigure::updateConfigData(const dynamic_reconfigure::Config &config)
{
  for (unsigned int i = 0; i < config.ints.size(); ++i)
  {
    if (!assignValue(registered_int_, config.ints[i].name, config.ints[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << config.ints[i].name << " not registered");
    }
  }
  for (unsigned int i = 0; i < config.doubles.size(); ++i)
  {
    if (!assignValue(registered_double_, config.doubles[i].name, config.doubles[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << config.doubles[i].name << " not registered");
    }
  }
  for (unsigned int i = 0; i < config.bools.size(); ++i)
  {
    if (!assignValue(registered_bool_, config.bools[i].name, config.bools[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << config.bools[i].name << " not registered");
    }
  }
  for (unsigned int i = 0; i < config.strs.size(); ++i)
  {
    if (!assignValue(registered_string_, config.strs[i].name, config.strs[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << config.strs[i].name << " not registered");
    }
  }
}

void DDynamicReconfigure::setUserCallback(const DDynamicReconfigure::UserCallbackType &callback)
{
  user_callback_ = callback;
}

void DDynamicReconfigure::clearUserCallback()
{
  user_callback_.clear();
}

void DDynamicReconfigure::RegisterVariable(double *variable, std::string id, double min, double max)
{
  registerVariable(id, variable, "", min, max);
}

void DDynamicReconfigure::RegisterVariable(int *variable, std::string id, int min, int max)
{
  registerVariable(id, variable, "", min, max);
}

void DDynamicReconfigure::RegisterVariable(bool *variable, std::string id)
{
  registerVariable(id, variable, "");
}

dynamic_reconfigure::ConfigDescription DDynamicReconfigure::generateConfigDescription() const
{
  dynamic_reconfigure::ConfigDescription config_description;

  std::map<std::string, dynamic_reconfigure::Group> groups;

  for (unsigned int i = 0; i < registered_int_.size(); ++i)
  {
    const RegisteredParam<int> &ri = *registered_int_[i];
    dynamic_reconfigure::ParamDescription p = ri.getParamDescription();

    auto &gp = groups[ri.group_];
    gp.parameters.push_back(p);

    // Max min def
    dynamic_reconfigure::IntParameter ip;
    ip.name = ri.name_;
    ip.value = ri.getCurrentValue();
    config_description.dflt.ints.push_back(ip);
    ip.value = ri.max_value_;
    config_description.max.ints.push_back(ip);
    ip.value = ri.min_value_;
    config_description.min.ints.push_back(ip);
  }

  for (unsigned int i = 0; i < registered_double_.size(); ++i)
  {
    const RegisteredParam<double> &rd = *registered_double_[i];
    dynamic_reconfigure::ParamDescription p = rd.getParamDescription();

    auto &gp = groups[rd.group_];
    gp.parameters.push_back(p);
    // Max min def
    dynamic_reconfigure::DoubleParameter dp;
    dp.name = rd.name_;
    dp.value = rd.getCurrentValue();
    config_description.dflt.doubles.push_back(dp);
    dp.value = rd.max_value_;
    config_description.max.doubles.push_back(dp);
    dp.value = rd.min_value_;
    config_description.min.doubles.push_back(dp);
  }

  for (unsigned int i = 0; i < registered_bool_.size(); ++i)
  {
    const RegisteredParam<bool> &rb = *registered_bool_[i];
    dynamic_reconfigure::ParamDescription p = rb.getParamDescription();

    auto &gp = groups[rb.group_];
    gp.parameters.push_back(p);

    // Max min def
    dynamic_reconfigure::BoolParameter bp;
    bp.name = rb.name_;
    bp.value = rb.getCurrentValue();
    config_description.dflt.bools.push_back(bp);
    bp.value = rb.max_value_;
    config_description.max.bools.push_back(bp);
    bp.value = rb.min_value_;
    config_description.min.bools.push_back(bp);
  }
  for (unsigned int i = 0; i < registered_string_.size(); ++i)
  {
    const RegisteredParam<std::string> &rs = *registered_string_[i];
    dynamic_reconfigure::ParamDescription p = rs.getParamDescription();

    auto &gp = groups[rs.group_];
    gp.parameters.push_back(p);

    // Max min def
    dynamic_reconfigure::StrParameter sp;
    sp.name = rs.name_;
    sp.value = rs.getCurrentValue();
    config_description.dflt.strs.push_back(sp);
    sp.value = rs.max_value_;
    config_description.max.strs.push_back(sp);
    sp.value = rs.min_value_;
    config_description.min.strs.push_back(sp);
  }

  auto default_group = groups["Default"];
  default_group.name = "Default";
  config_description.groups.push_back(default_group);

  dynamic_reconfigure::GroupState default_gs;
  default_gs.name = "Default";
  default_gs.state = true;
  config_description.dflt.groups.push_back(default_gs);
  config_description.min.groups.push_back(default_gs);
  config_description.max.groups.push_back(default_gs);

  size_t i = 1;
  for (auto &group : groups)
  {
    if (group.first == "Default")
    {
      continue;
    }
    group.second.name = group.first;
    group.second.type = "tab";
    group.second.id = i++;
    config_description.groups.push_back(group.second);

    dynamic_reconfigure::GroupState gs;
    gs.id = group.second.id;
    gs.name = group.second.name;
    gs.state = true;
    config_description.dflt.groups.push_back(gs);
    config_description.min.groups.push_back(gs);
    config_description.max.groups.push_back(gs);
  }
  return config_description;
}


dynamic_reconfigure::Config DDynamicReconfigure::generateConfig()
{
  dynamic_reconfigure::Config c;

  for (unsigned int i = 0; i < registered_int_.size(); ++i)
  {
    dynamic_reconfigure::IntParameter ip;
    ip.name = registered_int_[i]->name_;
    ip.value = registered_int_[i]->getCurrentValue();
    c.ints.push_back(ip);
  }

  for (unsigned int i = 0; i < registered_double_.size(); ++i)
  {
    dynamic_reconfigure::DoubleParameter dp;
    dp.name = registered_double_[i]->name_;
    dp.value = registered_double_[i]->getCurrentValue();
    c.doubles.push_back(dp);
  }

  for (unsigned int i = 0; i < registered_bool_.size(); ++i)
  {
    dynamic_reconfigure::BoolParameter bp;
    bp.name = registered_bool_[i]->name_;
    bp.value = registered_bool_[i]->getCurrentValue();
    c.bools.push_back(bp);
  }

  for (unsigned int i = 0; i < registered_string_.size(); ++i)
  {
    dynamic_reconfigure::StrParameter bs;
    bs.name = registered_string_[i]->name_;
    bs.value = registered_string_[i]->getCurrentValue();
    c.strs.push_back(bs);
  }

  for (size_t i = 0; i < config_groups_.size(); ++i)
  {
    dynamic_reconfigure::GroupState gs;
    gs.name = config_groups_[i];
    gs.id = i;
    gs.state = true;
    c.groups.push_back(gs);
  }

  return c;
}



void DDynamicReconfigure::PublishServicesTopics()
{
  publishServicesTopics();
}

void DDynamicReconfigure::updateRegisteredVariablesData()
{
  if (!new_config_avail_ || auto_update_)
    return;
  updateConfigData(updated_config_);
  new_config_avail_ = false;
}


// Explicit int instantations
template void DDynamicReconfigure::registerVariable(const std::string &name, int *variable,
                                                    const std::string &description, int min,
                                                    int max, const std::string &group);


template void DDynamicReconfigure::registerEnumVariable(const std::string &name, int *variable,
                                                        const std::string &description,
                                                        std::map<std::string, int> enum_dict,
                                                        const std::string &enum_description,
                                                        const std::string &group);
template void DDynamicReconfigure::registerVariable(const std::string &name, int current_value,
                                                    const boost::function<void(int value)> &callback,
                                                    const std::string &description, int min,
                                                    int max, const std::string &group);

template void DDynamicReconfigure::registerEnumVariable(
    const std::string &name, int current_value, const boost::function<void(int)> &callback,
    const std::string &description, std::map<std::string, int> enum_dict,
    const std::string &enum_description, const std::string &group);


// Explicit double instantations
template void DDynamicReconfigure::registerVariable(const std::string &name, double *variable,
                                                    const std::string &description, double min,
                                                    double max, const std::string &group);


template void DDynamicReconfigure::registerEnumVariable(const std::string &name, double *variable,
                                                        const std::string &description,
                                                        std::map<std::string, double> enum_dict,
                                                        const std::string &enum_description,
                                                        const std::string &group);
template void DDynamicReconfigure::registerVariable(
    const std::string &name, double current_value,
    const boost::function<void(double value)> &callback, const std::string &description,
    double min, double max, const std::string &group);

template void DDynamicReconfigure::registerEnumVariable(
    const std::string &name, double current_value, const boost::function<void(double)> &callback,
    const std::string &description, std::map<std::string, double> enum_dict,
    const std::string &enum_description, const std::string &group);



// Explicit bool instantations
template void DDynamicReconfigure::registerVariable(const std::string &name, bool *variable,
                                                    const std::string &description, bool min,
                                                    bool max, const std::string &group);


template void DDynamicReconfigure::registerEnumVariable(const std::string &name, bool *variable,
                                                        const std::string &description,
                                                        std::map<std::string, bool> enum_dict,
                                                        const std::string &enum_description,
                                                        const std::string &group);
template void DDynamicReconfigure::registerVariable(const std::string &name, bool current_value,
                                                    const boost::function<void(bool value)> &callback,
                                                    const std::string &description, bool min,
                                                    bool max, const std::string &group);

template void DDynamicReconfigure::registerEnumVariable(
    const std::string &name, bool current_value, const boost::function<void(bool)> &callback,
    const std::string &description, std::map<std::string, bool> enum_dict,
    const std::string &enum_description, const std::string &group);


// Explicit std::string instantations
template void DDynamicReconfigure::registerVariable(const std::string &name, std::string *variable,
                                                    const std::string &description,
                                                    std::string min, std::string max,
                                                    const std::string &group);


template void DDynamicReconfigure::registerEnumVariable(
    const std::string &name, std::string *variable, const std::string &description,
    std::map<std::string, std::string> enum_dict, const std::string &enum_description,
    const std::string &group);
template void DDynamicReconfigure::registerVariable(
    const std::string &name, std::string current_value,
    const boost::function<void(std::string value)> &callback, const std::string &description,
    std::string min, std::string max, const std::string &group);

template void DDynamicReconfigure::registerEnumVariable(
    const std::string &name, std::string current_value,
    const boost::function<void(std::string)> &callback, const std::string &description,
    std::map<std::string, std::string> enum_dict, const std::string &enum_description,
    const std::string &group);
}
