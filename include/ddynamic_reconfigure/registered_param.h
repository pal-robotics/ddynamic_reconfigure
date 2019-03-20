/*
  @file
  
  @author victor
  
  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/
#ifndef REGISTERED_PARAM_H
#define REGISTERED_PARAM_H

#include <string>
#include <map>
#include <sstream>
#include <dynamic_reconfigure/ParamDescription.h>
namespace ddynamic_reconfigure {
   
template <typename T>
class RegisteredParam
{
public:
  RegisteredParam(const std::string &name, const std::string &description, T min_value,
                  T max_value, const std::string &type_name,
                  std::map<std::string, T> enum_dictionary = {},
                  const std::string &enum_description = "")
    : name_(name)
    , description_(description)
    , min_value_(min_value)
    , max_value_(max_value)
    , type_name_(type_name)
    , enum_dictionary_(enum_dictionary)
    , enum_description_(enum_description)
  {
  }
  
  virtual ~RegisteredParam()
  {}

  virtual T getCurrentValue() const = 0;
  virtual void updateValue(T new_value) = 0;
  
  virtual dynamic_reconfigure::ParamDescription getParamDescription() const 
  {
    dynamic_reconfigure::ParamDescription p;
    p.name = name_;
    p.description = description_;
    p.level = 0;
    p.type = type_name_;
    if (!enum_dictionary_.empty())
    {
      p.edit_method = getEditMethod();
    }
    return p;
  }

  std::string getEditMethod() const
  {
    // Based on https://github.com/awesomebytes/ddynamic_reconfigure's implementation
    std::stringstream ret;
    ret << "{";
    {
      ret << "'enum_description': '" << enum_description_ << "', ";
      ret << "'enum': [";
      {
        auto it = enum_dictionary_.cbegin();
        ret << makeConst(it->first, it->second, "");
        for (it++; it != enum_dictionary_.cend(); it++)
        {
          ret << ", " << makeConst(it->first, it->second, "");
        };
      }
      ret << "]";
    }
    ret << "}";
    return ret.str();
  }

  std::string makeConst(const std::string &name, int value, const std::string &desc) const
  {
    std::stringstream ret;
    ret << "{";
    {
      ret << "'srcline': 0, ";  // the sole reason this is here is because dynamic placed
                                // it in its enum JSON.
      ret << "'description': '" << desc << "', ";
      ret << "'srcfile': '/does/this/really/matter.cfg', ";  // the answer is no. This is
                                                             // useless.
      ret << "'cconsttype': 'const " << type_name_ << "', ";
      ret << "'value': " << value << ", ";
      ret << "'ctype': '" << type_name_ << "', ";
      ret << "'type': '" << type_name_ << "', ";
      ret << "'name': '" << name << "'";
    }
    ret << "}";
    return ret.str();
  }

  const std::string name_;
  const std::string description_;
  const T min_value_;
  const T max_value_;
  const std::string type_name_;
  const std::map<std::string, T> enum_dictionary_;
  const std::string enum_description_;
};

} // namespace ddynamic_reconfigure

#endif // REGISTERED_PARAM_H
