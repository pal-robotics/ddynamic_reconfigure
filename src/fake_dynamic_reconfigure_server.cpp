#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <ddynamic_reconfigure/TutorialsConfig.h>

class DDynamicReconfigure{
    /**
      WORKFLOW:


    */
public:
    DDynamicReconfigure(const ros::NodeHandle &nh = ros::NodeHandle("~")):node_handle_(nh), advertized_(false){

    }

    bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                           dynamic_reconfigure::Reconfigure::Response &rsp)
    {
        ROS_INFO_STREAM("Called config callback");
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
        std::cerr<<req.config<<std::endl;

        update_pub_.publish(configMessage_);
        return true;
    }

    void RegisterVariable(int *variable, std::string id);
    void RegisterVariable(double *variable, std::string id);
    void RegisterVariable(bool *variable, std::string id);

    void generateConfigDescription(){
        dynamic_reconfigure::Group gp;

        gp.name = "default";
        for(unsigned int i=0; i<registered_int_.size(); ++i){
            dynamic_reconfigure::ParamDescription p;
            p.name  = registered_int_[i].first;
            //p.description = registered_int_[i].first;
            p.level = 0;
            p.type = "int";
            gp.parameters.push_back(p);
            //Max min def
            dynamic_reconfigure::IntParameter ip;
            ip.name = registered_int_[i].first;
            ip.value = *registered_int_[i].second;
            configDescription_.dflt.ints.push_back(ip);
            ip.value = 100;
            configDescription_.max.ints.push_back(ip);
            ip.value = -100;
            configDescription_.min.ints.push_back(ip);
        }

        for(unsigned int i=0; i<registered_double_.size(); ++i){
            dynamic_reconfigure::ParamDescription p;
            p.name  = registered_double_[i].first;
            //p.description = registered_double_[i].first;
            p.level = 0;
            p.type = "double";
            gp.parameters.push_back(p);
            //Max min def
            dynamic_reconfigure::DoubleParameter dp;
            dp.name = registered_double_[i].first;
            dp.value = *registered_double_[i].second;
            configDescription_.dflt.doubles.push_back(dp);
            dp.value = 100.0;
            configDescription_.max.doubles.push_back(dp);
            dp.value = -100.0;
            configDescription_.min.doubles.push_back(dp);
        }

        for(unsigned int i=0; i<registered_bool_.size(); ++i){
            dynamic_reconfigure::ParamDescription p;
            p.name  = registered_bool_[i].first;
            //p.description  = registered_bool_[i].first;
            p.level = 0;
            p.type = "bool";
            gp.parameters.push_back(p);
            //Max min def
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

    void generateConfig(){
        dynamic_reconfigure::Config c;

        dynamic_reconfigure::GroupState gs;
        gs.name = "Default";
        gs.state = true;
        c.groups.push_back(gs);

        for(unsigned int i=0; i<registered_int_.size(); ++i){
            dynamic_reconfigure::IntParameter ip;
            ip.name = registered_int_[i].first;
            ip.value = *registered_int_[i].second;
            c.ints.push_back(ip);
        }

        for(unsigned int i=0; i<registered_double_.size(); ++i){
            dynamic_reconfigure::DoubleParameter dp;
            dp.name = registered_double_[i].first;
            dp.value = *registered_double_[i].second;
            c.doubles.push_back(dp);
        }

        for(unsigned int i=0; i<registered_bool_.size(); ++i){
            dynamic_reconfigure::BoolParameter bp;
            bp.name = registered_bool_[i].first;
            bp.value = *registered_double_[i].second;
            c.bools.push_back(bp);
        }

    }

    void PublishServicesTopics(){
        set_service_ = node_handle_.advertiseService("set_parameters",
                                                     &DDynamicReconfigure::setConfigCallback, this);

        descr_pub_ = node_handle_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
        generateConfigDescription();
        descr_pub_.publish(configDescription_);

        update_pub_ = node_handle_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
        generateConfig();
        update_pub_.publish(configMessage_);

        advertized_ = true;
    }


private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer set_service_;
    ros::Publisher update_pub_;
    ros::Publisher descr_pub_;

    bool advertized_;

    //Registered variables
    std::vector< std::pair<std::string, int*> > registered_int_;
    std::vector< std::pair<std::string, double*> > registered_double_;
    std::vector<std::pair<std::string, bool*> > registered_bool_;

    dynamic_reconfigure::ConfigDescription configDescription_;
    dynamic_reconfigure::Config configMessage_;
};

void DDynamicReconfigure::RegisterVariable(int *variable, std::string id){
    std::pair<std::string, int*> p(id, variable);
    registered_int_.push_back(p);
}

void DDynamicReconfigure::RegisterVariable(double *variable, std::string id){
    std::pair<std::string, double*> p(id, variable);
    registered_double_.push_back(p);
}

void DDynamicReconfigure::RegisterVariable(bool *variable, std::string id){
    std::pair<std::string, bool*> p(id, variable);
    registered_bool_.push_back(p);
}

/**
  Topics:
  * /dynamic_tutorials/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
  * /dynamic_tutorials/parameter_updates [dynamic_reconfigure/Config]

  Services:
  * /dynamic_tutorials/set_parameter:  dynamic_reconfigure/Reconfigure
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_dynamic_reconfigure");

    double double_test = 0.0;
    int int_test = 0;
    bool bool_test = false;

    DDynamicReconfigure ddr;
    ddr.RegisterVariable(&double_test, "double_test");
    ddr.RegisterVariable(&int_test, "int_test");
    ddr.RegisterVariable(&bool_test, "bool_test");

    ddr.PublishServicesTopics();

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}

