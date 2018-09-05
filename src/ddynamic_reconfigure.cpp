#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace ddynamic_reconfigure
{
template <class T>
bool assignValue(std::vector<std::pair<std::string, T *> > v, std::string name, T value)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i].first == name)
    {
      *v[i].second = value;
      return true;
    }
  }
  return false;
}

template <class T, class V>
bool assignValue(std::vector<T> v, std::string name, V value)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i].name == name)
    {
      *v[i].value = value;
      return true;
    }
  }
  return false;
}

DDynamicReconfigure::DDynamicReconfigure(const ros::NodeHandle &nh, bool spin_thread)
  : node_handle_(nh), advertized_(false)
{
  if (spin_thread)
  {
    queued_nh_.setCallbackQueue(&queue_);
    spinner_.reset(new ros::AsyncSpinner(1, &queue_));
    spinner_->start();
  }
  else
  {
    queued_nh_ = nh;
  }
  pub_config_timer_ = queued_nh_.createTimer(ros::Duration(1.0), boost::bind(&DDynamicReconfigure::updatePublishedInformation, this)) ;
}

DDynamicReconfigure::~DDynamicReconfigure()
{
  set_service_.shutdown();
  update_pub_.shutdown();
  descr_pub_.shutdown();
}

void DDynamicReconfigure::updatePublishedInformation()
{
  dynamic_reconfigure::Config config_msg = generateConfig();
  update_pub_.publish(config_msg);
}

bool DDynamicReconfigure::setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                                            dynamic_reconfigure::Reconfigure::Response &rsp)
{
  ROS_DEBUG_STREAM("Called config callback of ddynamic_reconfigure");

  for (unsigned int i = 0; i < req.config.ints.size(); ++i)
  {
    if (!assignValue(registered_int_, req.config.ints[i].name, req.config.ints[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << req.config.ints[i].name << " not registered");
    }
  }
  for (unsigned int i = 0; i < req.config.doubles.size(); ++i)
  {
    if (!assignValue(registered_double_, req.config.doubles[i].name, req.config.doubles[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << req.config.doubles[i].name << " not registered");
    }
  }
  for (unsigned int i = 0; i < req.config.bools.size(); ++i)
  {
    if (!assignValue<bool>(registered_bool_, req.config.bools[i].name, req.config.bools[i].value))
    {
      ROS_ERROR_STREAM("Variable :" << req.config.bools[i].name << " not registered");
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

  return true;
}

void DDynamicReconfigure::setUserCallback(const DDynamicReconfigure::UserCallbackType &callback)
{
  user_callback_ = callback;
}

void DDynamicReconfigure::clearUserCallback()
{
  user_callback_.clear();
}

void DDynamicReconfigure::generateConfigDescription()
{
  dynamic_reconfigure::Group gp;

  gp.name = "default";
  for (unsigned int i = 0; i < registered_int_.size(); ++i)
  {
    dynamic_reconfigure::ParamDescription p;
    p.name = registered_int_[i].name;
    // p.description = registered_int_[i].first;
    p.level = 0;
    p.type = "int";
    gp.parameters.push_back(p);
    // Max min def
    dynamic_reconfigure::IntParameter ip;
    ip.name = registered_int_[i].name;
    ip.value = *registered_int_[i].value;
    configDescription_.dflt.ints.push_back(ip);
    ip.value = registered_int_[i].max_value;
    configDescription_.max.ints.push_back(ip);
    ip.value = -registered_int_[i].min_value;
    configDescription_.min.ints.push_back(ip);
  }

  for (unsigned int i = 0; i < registered_double_.size(); ++i)
  {
    dynamic_reconfigure::ParamDescription p;
    p.name = registered_double_[i].name;
    // p.description = registered_double_[i].first;
    p.level = 0;
    p.type = "double";
    gp.parameters.push_back(p);
    // Max min def
    dynamic_reconfigure::DoubleParameter dp;
    dp.name = registered_double_[i].name;
    dp.value = *registered_double_[i].value;
    configDescription_.dflt.doubles.push_back(dp);
    dp.value = registered_double_[i].max_value;
    configDescription_.max.doubles.push_back(dp);
    dp.value = registered_double_[i].min_value;
    configDescription_.min.doubles.push_back(dp);
  }

  for (unsigned int i = 0; i < registered_bool_.size(); ++i)
  {
    dynamic_reconfigure::ParamDescription p;
    p.name = registered_bool_[i].first;
    // p.description  = registered_bool_[i].first;
    p.level = 0;
    p.type = "bool";
    gp.parameters.push_back(p);
    // Max min def
    dynamic_reconfigure::BoolParameter bp;
    bp.name = registered_bool_[i].first;
    bp.value = *registered_bool_[i].second;
    configDescription_.dflt.bools.push_back(bp);
    bp.value = true;
    configDescription_.max.bools.push_back(bp);
    bp.value = false;
    configDescription_.min.bools.push_back(bp);
  }
  configDescription_.groups.push_back(gp);
}


dynamic_reconfigure::Config DDynamicReconfigure::generateConfig()
{
  dynamic_reconfigure::Config c;

  dynamic_reconfigure::GroupState gs;
  gs.name = "Default";
  gs.state = true;
  c.groups.push_back(gs);

  for (unsigned int i = 0; i < registered_int_.size(); ++i)
  {
    dynamic_reconfigure::IntParameter ip;
    ip.name = registered_int_[i].name;
    ip.value = *registered_int_[i].value;
    c.ints.push_back(ip);
  }

  for (unsigned int i = 0; i < registered_double_.size(); ++i)
  {
    dynamic_reconfigure::DoubleParameter dp;
    dp.name = registered_double_[i].name;
    dp.value = *registered_double_[i].value;
    c.doubles.push_back(dp);
  }

  for (unsigned int i = 0; i < registered_bool_.size(); ++i)
  {
    dynamic_reconfigure::BoolParameter bp;
    bp.name = registered_bool_[i].first;
    bp.value = *registered_bool_[i].second;
    c.bools.push_back(bp);
  }

  return c;
}

void DDynamicReconfigure::PublishServicesTopics()
{
  set_service_ = node_handle_.advertiseService("set_parameters",
                                               &DDynamicReconfigure::setConfigCallback, this);

  descr_pub_ = node_handle_.advertise<dynamic_reconfigure::ConfigDescription>(
      "parameter_descriptions", 1, true);
  generateConfigDescription();
  descr_pub_.publish(configDescription_);

  update_pub_ =
      node_handle_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

  update_pub_.publish(generateConfig());

  advertized_ = true;
}

void DDynamicReconfigure::RegisterVariable(int *variable, std::string id)
{
  RegisteredInt p;
  p.name = id;
  p.value = variable;
  registered_int_.push_back(p);
  if (node_handle_.hasParam(id))
  {
    node_handle_.param<int>(id, *variable, 0);
  }
}

void DDynamicReconfigure::RegisterVariable(int *variable, std::string id, double min, double max)
{
  RegisteredInt p;
  p.name = id;
  p.value = variable;
  p.min_value = min;
  p.max_value = max;
  registered_int_.push_back(p);
  if (node_handle_.hasParam(id))
  {
    node_handle_.param<int>(id, *variable, 0);
  }
}

void DDynamicReconfigure::RegisterVariable(double *variable, std::string id)
{
  RegisteredDouble p;
  p.name = id;
  p.value = variable;
  registered_double_.push_back(p);
  if (node_handle_.hasParam(id))
  {
    node_handle_.param<double>(id, *variable, 0.0);
  }
}

void DDynamicReconfigure::RegisterVariable(double *variable, std::string id, double min, double max)
{
  RegisteredDouble p;
  p.name = id;
  p.value = variable;
  p.min_value = min;
  p.max_value = max;
  registered_double_.push_back(p);
  if (node_handle_.hasParam(id))
  {
    node_handle_.param<double>(id, *variable, 0.0);
  }
}

void DDynamicReconfigure::RegisterVariable(bool *variable, std::string id)
{
  std::pair<std::string, bool *> p(id, variable);
  registered_bool_.push_back(p);
  if (node_handle_.hasParam(id))
  {
    node_handle_.param<bool>(id, *variable, false);
  }
}
}
