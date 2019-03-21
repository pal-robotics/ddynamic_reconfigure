/*
  @file
  
  @author victor
  
  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/
#ifndef DDYNAMIC_RECONFIGURE_UTILS_H
#define DDYNAMIC_RECONFIGURE_UTILS_H
#include <string>
#include <limits>
#include <vector>

template <typename T>
inline T getMin()
{
  return std::numeric_limits<T>::min();
}

template <>
inline bool getMin()
{
  return false;
}

template <>
inline std::string getMin<std::string>()
{
  return "";
}

template <typename T>
inline T getMax()
{
  return std::numeric_limits<T>::min();
}

template <>
inline bool getMax()
{
  return true;
}

template <>
inline std::string getMax()
{
  return "";
}


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

template <typename T>
void attemptGetParam(ros::NodeHandle &nh, const std::string &name, T &param, T default_value)
{
  if (nh.hasParam(name))
  {
    nh.param<T>(name, param, default_value);
  }
}
template <typename T>
std::pair<T, T> getMinMax(const std::map<std::string, T> &enum_map)
{
  T min, max;
  if (enum_map.empty())
  {
    throw std::runtime_error("Trying to register an empty enum");
  }
  
  min = enum_map.begin()->second;
  max = enum_map.begin()->second;
  
  for (const auto &it : enum_map)
  {
    min = std::min(min, it.second);
    max = std::max(min, it.second);
  }
  
  return std::make_pair(min, max);
}



#endif // DDYNAMIC_RECONFIGURE_UTILS_H
