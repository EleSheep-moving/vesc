#include "aims_longitudinal_control/control.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
using namespace aims_longitudinal_control;
bool near(double a,double b){return std::abs(a-b)<1e-9;}
int main(){
  Config off;ReferenceSample target{3.,0.,true,""};
  const auto plain=control(off,target,3.);
  assert(!off.steering_compensation_enabled);
  assert(off.steering_compensation_max_current==20.);
  auto on=off;on.steering_compensation_enabled=true;
  assert(on.identity()!=off.identity());
  auto inactive=off;inactive.steering_compensation_gain=5.;assert(inactive.identity()==off.identity());
  const double angle=.3490658503988659;
  const auto disabled=control(off,target,3.,0.,false,angle);
  assert(disabled.steering==0.&&near(disabled.current,plain.current));
  const auto left=control(on,target,3.,0.,false,angle);
  const auto right=control(on,target,3.,0.,false,-angle);
  const double expected=on.steering_compensation_gain*9.*std::pow(std::tan(angle),2);
  assert(near(left.steering,expected)&&near(left.steering,right.steering));
  assert(near(left.feedforward,plain.feedforward)&&near(left.proportional,plain.proportional));
  assert(near(left.unclamped,plain.unclamped+expected));
  auto with_d=on;with_d.ka=.2;
  const auto four=control(with_d,target,2.8,.3,true,angle);
  assert(four.proportional>0.&&four.derivative<0.&&four.steering>0.);
  assert(near(four.unclamped,four.feedforward+four.steering+four.proportional+four.derivative));
  assert(control(on,target,0.,0.,false,angle).steering==0.);
  assert(control(on,target,-1.,0.,false,angle).steering==0.);
  assert(control(on,target,-1.,0.,false,angle).steering_limited);
  assert(control(on,target,3.,0.,false,0.).steering==0.);
  const auto high=control(on,target,100.,0.,false,100.);
  assert(near(high.steering,on.steering_compensation_max_current)&&high.steering_limited);
  auto limit=on;limit.current_max=12.;
  const auto capped=control(limit,target,3.,0.,false,angle);
  assert(capped.current==12.&&capped.limited&&capped.steering>0.);
  auto braking=target;braking.acceleration=-1.;
  const auto decel=control(on,braking,3.,0.,false,angle);
  const auto decel_off=control(off,braking,3.,0.,false,angle);
  assert(near(decel.steering,expected)&&decel.unclamped>decel_off.unclamped);
  // The nominal plant includes the same estimated load: no fictitious net
  // acceleration from a compensation current; current saturation loses headroom.
  assert(near(nominal_acceleration(on,target,3.,0.,false,angle),0.));
  assert(nominal_acceleration(limit,target,3.,0.,false,angle)<0.);
  for(int field=0;field<3;++field){
    auto bad=on;
    if(field==0)bad.steering_compensation_gain=-1.;
    if(field==1)bad.steering_compensation_max_current=-1.;
    if(field==2)bad.steering_compensation_max_angle=1.6;
    bool rejected=false;try{bad.validate();}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  bool rejected=false;
  try{control(on,target,3.,0.,false,std::numeric_limits<double>::quiet_NaN());}
  catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  std::cout<<"steering compensation contracts PASS\n";
}
