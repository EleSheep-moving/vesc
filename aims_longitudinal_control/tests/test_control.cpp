#include "aims_longitudinal_control/control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace aims_longitudinal_control;
bool near(double a,double b){return std::abs(a-b)<1e-9;}
int main(){
  Config c;c.validate();assert(c.kv>0&&c.ka==0);
  auto id=c.identity();auto other=c;other.kv=1.;assert(other.identity()!=id);
  bool rejected=false;try{other.kv=0;other.validate();}catch(...){rejected=true;}assert(rejected);
  auto b=feedforward(c,2.);auto coast=feedforward(c,-1.);
  assert(near(b.feedforward,(2.-c.b_b)/c.k_b)&&b.weight==1.);
  assert(near(coast.feedforward,(-1.-c.b_c)/c.k_c)&&coast.weight==0.);
  double previous=-1e9;
  for(int i=0;i<=2000;++i){double a=-2.+i*.002;
    auto f=feedforward(c,a);assert(f.feedforward>=previous);previous=f.feedforward;
    assert(near(f.gain,(1-f.weight)/c.k_c+f.weight/c.k_b));}
  for(double edge:{c.blend_low,c.blend_high})
    assert(std::abs(feedforward(c,edge-1e-8).feedforward-feedforward(c,edge+1e-8).feedforward)<1e-5);
  Reference r{7,10.,.02,10.1,3.,1.};assert(valid_reference(r,c));
  auto start=reference_sample(r,10.);auto mid=reference_sample(r,10.01);auto end=reference_sample(r,10.02);
  assert(start.valid&&near(start.speed,3.)&&near(start.acceleration,1.));
  assert(mid.valid&&near(mid.speed,3.01)&&near(mid.acceleration,1.));
  assert(end.valid&&near(end.speed,3.02)&&end.acceleration==0.);
  assert(near(reference_sample(r,10.05).speed,3.02));
  assert(!reference_sample(r,10.1001).valid&&!reference_sample(r,9.99).valid);
  auto expanded=c;expanded.accel_max=2.;auto wide=r;wide.acceleration=2.;
  assert(valid_reference(wide,expanded)&&reference_sample(wide,10.,expanded).valid);
  for(auto pair:{std::pair<double,double>{4.98,1.},std::pair<double,double>{.03,-1.5}}){
    auto raw=r;raw.speed=pair.first;raw.acceleration=pair.second;
    auto wire=canonical_reference(raw,c);
    wire.speed=static_cast<float>(wire.speed);wire.acceleration=static_cast<float>(wire.acceleration);
    assert(valid_reference(wire,c));assert(wire.speed+wire.acceleration*wire.duration<=5.);
    assert(wire.speed+wire.acceleration*wire.duration>=0.);
  }
  auto p=control(c,mid,2.9);auto fast=control(c,mid,3.2);
  assert(p.proportional>0&&fast.proportional<0&&p.derivative==0);
  assert(near(p.proportional,c.kv*p.gain*(mid.speed-2.9)));
  assert(near(control(c,mid,-100.).current,30.));assert(near(control(c,mid,100.).current,-15.));
  c.ka=.2;auto d=control(c,mid,mid.speed,2.,true);assert(d.derivative<0);
  assert(control(c,mid,mid.speed,2.,false).derivative==0.);
  WheelDerivative wheel(c);assert(wheel.observe(1.,1.)&&!wheel.valid());
  assert(wheel.observe(1.01,1.02)&&wheel.valid()&&near(wheel.acceleration(),2.));
  auto a=wheel.acceleration();assert(!wheel.observe(1.01,9.)&&near(wheel.acceleration(),a));
  assert(wheel.observe(1.02,1.02)&&wheel.acceleration()>0&&wheel.acceleration()<2.);
  assert(wheel.observe(1.1,1.1)&&!wheel.valid());
  assert(!wheel.observe(1.09,1.)&&!wheel.valid());
  wheel.reset();assert(!wheel.valid());
  c.ka=0.;auto f=control(c,start,2.9);
  assert(near(nominal_acceleration(c,start,2.9),start.acceleration+(f.current-f.feedforward)/f.gain));
  // Closed loop nominal plant, including constant unmodelled acceleration.
  for(double bias:{-.5,.5}){double v=0.;
    for(int i=0;i<3000;++i){double target=i<1000?1.:(i<2000?2.:1.);ReferenceSample s{target,0.,true,""};
      double a_nom=nominal_acceleration(c,s,v);v+=.01*(a_nom+bias);
      assert(std::isfinite(v)&&std::abs(v)<3.);}
    assert(std::abs(v-1.)<.251);}
  // A continuous acceleration/deceleration reference with persistent model bias.
  for(double bias:{-.5,0.,.5}){double v=1.,target=1.;
    for(int i=0;i<1200;++i){double a_ref=i<400?.5:(i<800?-.5:0.);
      ReferenceSample s{target,a_ref,true,""};
      const auto terms=control(c,s,v);
      assert(terms.current>=c.current_min&&terms.current<=c.current_max);
      v+=.01*(nominal_acceleration(c,s,v)+bias);target+=.01*a_ref;
      assert(std::isfinite(v)&&std::abs(v-target)<.26);
    }
  }
  std::cout<<"longitudinal control contracts PASS\n";
}
