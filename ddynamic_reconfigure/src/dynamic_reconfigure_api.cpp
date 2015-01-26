#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config conf;

double_param.name = "kurtana_pitch_joint";
double_param.value = pitch;
conf.doubles.push_back(double_param);

double_param.name = "kurtana_roll_joint";
double_param.value = yaw;
conf.doubles.push_back(double_param);

srv_req.config = conf;

ros::service::call("/joint_commander/set_parameters", srv_req, 
srv_resp);
