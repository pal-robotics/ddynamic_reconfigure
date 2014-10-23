#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

class DDynamicReconfigure{
  /**
      WORKFLOW:
      
    */
public:
  DDynamicReconfigure(const ros::NodeHandle &nh = ros::NodeHandle("~"));

  void RegisterVariable(int *variable, std::string id);

  void RegisterVariable(double *variable, std::string id);

  void RegisterVariable(bool *variable, std::string id);

  void generateConfigDescription();

  void generateConfig();

  void PublishServicesTopics();

  void updatePublishedInformation();

  bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                                              dynamic_reconfigure::Reconfigure::Response &rsp);

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

