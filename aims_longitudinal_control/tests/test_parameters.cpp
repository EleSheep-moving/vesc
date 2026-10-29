#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include "aims_longitudinal_control/parameters.hpp"
int main(int argc,char** argv){
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("longitudinal_parameter_contract");
  auto cfg=aims_longitudinal_control::declare_parameters(*node);assert(cfg.kv>0);
  assert(!cfg.steering_compensation_enabled);
  assert(node->describe_parameter("longitudinal.steering_compensation_enabled").read_only);
  assert(!node->set_parameter(rclcpp::Parameter("longitudinal.steering_compensation_enabled",true)).successful);
  assert(node->describe_parameter("longitudinal.kv").read_only);
  const auto result=node->set_parameter(rclcpp::Parameter("longitudinal.kv",0.));
  assert(!result.successful&&node->get_parameter("longitudinal.kv").as_double()==cfg.kv);
  node.reset();rclcpp::shutdown();
}
