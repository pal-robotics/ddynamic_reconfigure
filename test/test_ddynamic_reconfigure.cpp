#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <gmock/gmock.h>
#include <dynamic_reconfigure/Reconfigure.h>

using ::testing::_;
using ::testing::Mock;
using ::testing::Exactly;

using namespace ddynamic_reconfigure;

namespace  pal
{
class MockClass
{
public:
  MockClass()
    : double_param_(0.0), int_param_(0), bool_param_(false)
  {}
  MOCK_METHOD0(userCallback,
               void());

  double double_param_;
  int int_param_;
  bool bool_param_;
};


TEST(DDynamicReconfigureTest, basicTest)
{
  ros::NodeHandle nh("~");
  DDynamicReconfigure dd(nh);
  MockClass mock;
  mock.int_param_ = 0;
  dd.RegisterVariable(&mock.int_param_, "int_param", -10000, 10000);

  dd.PublishServicesTopics();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 1234;

  srv.request.config.ints.push_back(int_param);
  EXPECT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
  EXPECT_EQ(mock.int_param_, int_param.value);
}

TEST(DDynamicReconfigureTest, callbackTest)
{
  ros::NodeHandle nh("~");
  DDynamicReconfigure dd(nh);
  MockClass mock;
  mock.int_param_ = 0;
  mock.bool_param_ = false;
  dd.RegisterVariable(&mock.int_param_, "int_param", 0, 100);
  dd.RegisterVariable(&mock.bool_param_, "bool_param");
  dd.RegisterVariable(&mock.double_param_, "double_param", -50, 50);
  dd.setUserCallback(boost::bind(&MockClass::userCallback, &mock));
  dd.PublishServicesTopics();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  EXPECT_CALL(mock,
              userCallback())
      .Times(Exactly(1));

  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = -1234;
  srv.request.config.ints.push_back(int_param);

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param";
  bool_param.value = true;
  srv.request.config.bools.push_back(bool_param);

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 42.4242;
  srv.request.config.doubles.push_back(double_param);

  EXPECT_NE(mock.int_param_, int_param.value);
  EXPECT_FALSE(mock.bool_param_);
  EXPECT_NE(mock.double_param_, double_param.value);
  EXPECT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
  EXPECT_TRUE(mock.bool_param_);
  EXPECT_EQ(mock.int_param_, int_param.value);
  EXPECT_NEAR(mock.double_param_, double_param.value, 0.0001);
}


}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ddynamic_reconfigure_test");
  ros::NodeHandle nh;

  ::testing::InitGoogleMock(&argc, argv);

  return RUN_ALL_TESTS();
}
