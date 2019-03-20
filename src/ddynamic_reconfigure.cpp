#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <boost/make_unique.hpp>
namespace ddynamic_reconfigure
{

template <class T, class V>
bool assignValue(std::vector<T> &v, std::string name, V value)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
    {
      v[i]->updateValue(value);
      return true;
    }
  }
  return false;
}

DDynamicReconfigure::DDynamicReconfigure(const ros::NodeHandle &nh)
  : node_handle_(nh), advertized_(false)
{
  pub_config_timer_ = nh.createTimer(ros::Duration(5.0), boost::bind(&DDynamicReconfigure::updatePublishedInformation, this)) ;
}

DDynamicReconfigure::~DDynamicReconfigure()
{
  set_service_.shutdown();
  update_pub_.shutdown();
  descr_pub_.shutdown();
}

void DDynamicReconfigure::registerVariable(const std::string &name, int *variable, const std::string &description, int min, int max)
{
  registered_int_.push_back(boost::make_unique<PointerRegisteredParam<int>>(
      name, description, min, max, variable));
  if (node_handle_.hasParam(name))
  {
    node_handle_.param<int>(name, *variable, 0);
  } 
}

void DDynamicReconfigure::registerVariable(const std::string &name, double *variable, const std::string &description, double min, double max)
{
  registered_double_.push_back(boost::make_unique<PointerRegisteredParam<double>>(
      name, description, min, max, variable));
  if (node_handle_.hasParam(name))
  {
    node_handle_.param<double>(name, *variable, 0);
  } 
}

void DDynamicReconfigure::registerVariable(const std::string &name, bool *variable, const std::string &description)
{
  registered_bool_.push_back(boost::make_unique<PointerRegisteredParam<bool>>(
      name, description, false, true, variable));
  if (node_handle_.hasParam(name))
  {
    node_handle_.param<bool>(name, *variable, 0);
  } 
}

void DDynamicReconfigure::registerVariable(const std::string &name, int current_value, const boost::function<void (int)> &callback, const std::string &description, int min, int max)
{
  registered_int_.push_back(boost::make_unique<CallbackRegisteredParam<int>>(
      name, description, min, max, current_value, callback));
  if (node_handle_.hasParam(name))
  {
    node_handle_.param<int>(name, current_value, 0);
  } 
}

void DDynamicReconfigure::registerVariable(const std::string &name, double current_value, const boost::function<void (double)> &callback, const std::string &description, double min, double max)
{
  registered_double_.push_back(boost::make_unique<CallbackRegisteredParam<double>>(
      name, description, min, max, current_value, callback));
  if (node_handle_.hasParam(name))
  {
    node_handle_.param<double>(name, current_value, 0.0);
  }  
}

void DDynamicReconfigure::registerVariable(const std::string &name, bool current_value, const boost::function<void (bool)> &callback, const std::string &description)
{
  registered_bool_.push_back(boost::make_unique<CallbackRegisteredParam<bool>>(
      name, description, false, true, current_value, callback));
  if (node_handle_.hasParam(name))
  {
    node_handle_.param<bool>(name, current_value, false);
  }   
}
template <typename ParamType>
bool confCompare(const ParamType &a, const ParamType &b)
{
  return (a.name == b.name) && (a.value == b.value);
}

void DDynamicReconfigure::updatePublishedInformation()
{
  dynamic_reconfigure::Config config_msg = generateConfig();

  bool has_changed = false;
  has_changed = has_changed || config_msg.ints.size() != last_config_.ints.size();
  has_changed = has_changed || config_msg.doubles.size() != last_config_.doubles.size();
  has_changed = has_changed || config_msg.bools.size() != last_config_.bools.size();
  
  has_changed = has_changed || !std::equal(config_msg.ints.begin(), config_msg.ints.end(),
                             last_config_.ints.begin(),
                             confCompare<dynamic_reconfigure::IntParameter>);
  has_changed = has_changed || !std::equal(config_msg.doubles.begin(), config_msg.doubles.end(),
                            last_config_.doubles.begin(),
                            confCompare<dynamic_reconfigure::DoubleParameter>);
  has_changed = has_changed || !std::equal(config_msg.bools.begin(), config_msg.bools.end(),
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
    if (!assignValue(registered_bool_, req.config.bools[i].name, req.config.bools[i].value))
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
  
  pub_config_timer_.setPeriod(ros::Duration(5.0));
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

dynamic_reconfigure::ConfigDescription DDynamicReconfigure::generateConfigDescription() const
{
  dynamic_reconfigure::ConfigDescription config_description;
  dynamic_reconfigure::Group gp;

  gp.name = "default";
  for (unsigned int i = 0; i < registered_int_.size(); ++i)
  {
    const RegisteredParam<int> &ri = *registered_int_[i];
    dynamic_reconfigure::ParamDescription p;
    p.name = ri.name_;
    p.description = ri.description_;
    p.level = 0;
    p.type = "int";
    gp.parameters.push_back(p);
    // Max min def
    dynamic_reconfigure::IntParameter ip;
    ip.name = ri.name_;
    ip.value = ri.getCurrentValue();
    config_description.dflt.ints.push_back(ip);
    ip.value = ri.max_value_;
    config_description.max.ints.push_back(ip);
    ip.value = -ri.min_value_;
    config_description.min.ints.push_back(ip);
  }

  for (unsigned int i = 0; i < registered_double_.size(); ++i)
  {
    const RegisteredParam<double> &rd = *registered_double_[i];
    dynamic_reconfigure::ParamDescription p;
    p.name = rd.name_;
    p.description = rd.description_;
    p.level = 0;
    p.type = "double";
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
    dynamic_reconfigure::ParamDescription p;
    p.name = rb.name_;
    p.description = rb.description_;
    p.level = 0;
    p.type = "bool";
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
  config_description.groups.push_back(gp);
  return config_description;
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

  return c;
}

void DDynamicReconfigure::PublishServicesTopics()
{
  set_service_ = node_handle_.advertiseService("set_parameters",
                                               &DDynamicReconfigure::setConfigCallback, this);

  descr_pub_ = node_handle_.advertise<dynamic_reconfigure::ConfigDescription>(
      "parameter_descriptions", 1, true);
  const dynamic_reconfigure::ConfigDescription config_description = generateConfigDescription();
  descr_pub_.publish(config_description);

  update_pub_ =
      node_handle_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

  update_pub_.publish(generateConfig());

  advertized_ = true;
}

}
