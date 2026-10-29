// Copyright 2026 AIMSRacer. Licensed under the Apache License, Version 2.0.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace aims_longitudinal_control {
struct Config {
  double k_b=.10514677523013137,b_b=-1.3706685248564916;
  double k_c=.12936510607178806,b_c=-1.2503214337957151;
  double blend_low=-.71,blend_high=1.12,kv=2.,ka=0.;
  double derivative_tau=.03,source_gap=.05,wheel_max_age=.1;
  double current_min=-15.,current_max=30.,max_speed=5.,accel_min=-1.5,accel_max=1.;
  // Experimental additive turn-load current. Inputs remain low-level signals.
  bool steering_compensation_enabled=false;
  double steering_compensation_gain=3.9724336681547934;
  double steering_compensation_max_current=20.;
  double steering_compensation_max_angle=.4710920751094818;
  void validate() const {
    for(double x:{k_b,b_b,k_c,b_c,blend_low,blend_high,kv,ka,derivative_tau,
        source_gap,wheel_max_age,current_min,current_max,max_speed,accel_min,accel_max,
        steering_compensation_gain,steering_compensation_max_current,steering_compensation_max_angle})
      if(!std::isfinite(x)) throw std::invalid_argument("nonfinite longitudinal configuration");
    if(k_b<=0||k_c<=0||blend_low>=blend_high||kv<=0||ka<0||derivative_tau<=0||
       source_gap<=0||source_gap>.05||wheel_max_age<=0||wheel_max_age>.1||
       current_min>=0||current_max<=0||current_min < -100||current_max>100||
       max_speed<=0||max_speed>5||accel_min>=0||accel_max<=0||
       steering_compensation_gain<0||steering_compensation_max_current<0||
       steering_compensation_max_current>100||steering_compensation_max_angle<=0||
       steering_compensation_max_angle>=1.5707963267948966)
      throw std::invalid_argument("invalid longitudinal configuration; positive speed P required");
  }
  double clip_current(double current) const {
    if(!std::isfinite(current)) throw std::invalid_argument("nonfinite requested current");
    return std::clamp(current,current_min,current_max);
  }
  std::string identity() const {
    std::ostringstream canonical;canonical.imbue(std::locale::classic());canonical<<std::setprecision(17);
    canonical<<"bc_pd_v1";
    for(double x:{k_b,b_b,k_c,b_c,blend_low,blend_high,kv,ka,derivative_tau,
        source_gap,wheel_max_age,current_min,current_max,max_speed,accel_min,accel_max}) canonical<<':'<<x;
    // Disabled behavior/identity remains compatible with the original model.
    if(steering_compensation_enabled)
      canonical<<":steering_v1:"<<steering_compensation_gain<<':'<<steering_compensation_max_current
               <<':'<<steering_compensation_max_angle;
    // Stable configuration fingerprint, not a cryptographic provenance hash.
    std::uint64_t h=14695981039346656037ULL;
    for(unsigned char x:canonical.str()){h^=x;h*=1099511628211ULL;}
    std::ostringstream out;out<<"bc_pd_v1-"<<std::hex<<std::setfill('0')<<std::setw(16)<<h;return out.str();
  }
};
struct Reference {
  std::uint64_t plan_id{};
  double stamp{},duration{},valid_until{},speed{},acceleration{};
};
struct ReferenceSample {double speed{},acceleration{};bool valid{};std::string reason;};
inline double time_epsilon(double a,double b) {
  return 4*std::max(std::abs(std::nextafter(a,INFINITY)-a),std::abs(std::nextafter(b,INFINITY)-b));
}
inline bool valid_reference(const Reference& r,const Config& c=Config{}) {
  for(double x:{r.stamp,r.duration,r.valid_until,r.speed,r.acceleration})if(!std::isfinite(x))return false;
  const double epsilon=std::max(1e-9,time_epsilon(r.stamp,r.valid_until));
  return r.plan_id>0&&r.stamp>0&&r.duration>0&&r.duration<=.02+1e-9&&
    r.valid_until>r.stamp&&r.valid_until-r.stamp<=.1+epsilon&&
    r.speed>=0&&r.speed<=c.max_speed&&r.acceleration>=c.accel_min&&r.acceleration<=c.accel_max&&
    r.speed+r.acceleration*r.duration>=-1e-9&&r.speed+r.acceleration*r.duration<=c.max_speed+1e-9;
}
inline ReferenceSample reference_sample(const Reference& r,double now,const Config& config=Config{}) {
  if(!valid_reference(r,config)||!std::isfinite(now))return {0.,0.,false,"invalid_reference"};
  if(now<r.stamp)return {0.,0.,false,"future_reference"};
  if(now>=r.valid_until)return {0.,0.,false,"expired_reference"};
  const double elapsed=now-r.stamp;
  const double epsilon=std::max(1e-9,time_epsilon(now,r.stamp));
  const bool held=elapsed+epsilon>=r.duration;
  return {std::max(0.,r.speed+r.acceleration*std::min(elapsed,r.duration)),held?0.:r.acceleration,true,""};
}
// VescCommand stores float32 targets. Quantize before committing the sampler
// state, keeping the terminal speed admissible even at zero/max-speed edges.
inline Reference canonical_reference(Reference r,const Config& config=Config{}) {
  config.validate();
  if(!valid_reference(r,config))throw std::invalid_argument("invalid reference before wire quantization");
  float speed=static_cast<float>(r.speed);
  if(static_cast<double>(speed)>config.max_speed)speed=std::nextafter(speed,0.f);
  r.speed=speed;
  const double low=std::max(config.accel_min,-r.speed/r.duration);
  const double high=std::min(config.accel_max,(config.max_speed-r.speed)/r.duration);
  float acceleration=static_cast<float>(std::clamp(r.acceleration,low,high));
  if(static_cast<double>(acceleration)>high)acceleration=std::nextafter(acceleration,-INFINITY);
  if(static_cast<double>(acceleration)<low)acceleration=std::nextafter(acceleration,INFINITY);
  r.acceleration=acceleration;
  if(!valid_reference(r,config))throw std::invalid_argument("unrepresentable paired reference");
  return r;
}
struct Components {
  double feedforward{},proportional{},derivative{},unclamped{},current{},gain{},weight{};
  double steering{};
  bool transition{},limited{},steering_limited{};
};
inline Components feedforward(const Config& c,double acceleration) {
  c.validate();if(!std::isfinite(acceleration))throw std::invalid_argument("nonfinite reference acceleration");
  const double u=std::clamp((acceleration-c.blend_low)/(c.blend_high-c.blend_low),0.,1.);
  const double w=u*u*(3.-2.*u);
  Components result;result.weight=w;result.transition=acceleration>c.blend_low&&acceleration<c.blend_high;
  result.gain=(1.-w)/c.k_c+w/c.k_b;
  result.feedforward=(1.-w)*(acceleration-c.b_c)/c.k_c+w*(acceleration-c.b_b)/c.k_b;
  return result;
}
inline Components control(const Config& c,const ReferenceSample& r,double speed,
                          double acceleration=0.,bool derivative_valid=false,double steering_angle=0.) {
  if(!r.valid||!std::isfinite(speed)||!std::isfinite(r.speed)||
     !std::isfinite(steering_angle)||
     (derivative_valid&&!std::isfinite(acceleration)))throw std::invalid_argument("invalid longitudinal feedback/reference");
  auto out=feedforward(c,r.acceleration);
  if(c.steering_compensation_enabled){
    const double v=std::clamp(speed,0.,c.max_speed);
    const double angle=std::clamp(steering_angle,-c.steering_compensation_max_angle,
                                c.steering_compensation_max_angle);
    const double tangent=std::tan(angle);
    const double request=c.steering_compensation_gain*v*v*tangent*tangent;
    out.steering=std::min(request,c.steering_compensation_max_current);
    out.steering_limited=request>out.steering||angle!=steering_angle||v!=speed;
  }
  out.proportional=c.kv*out.gain*(r.speed-speed);
  out.derivative=derivative_valid?c.ka*out.gain*(r.acceleration-acceleration):0.;
  out.unclamped=out.feedforward+out.proportional+out.derivative+out.steering;
  out.current=c.clip_current(out.unclamped);out.limited=std::abs(out.current-out.unclamped)>1e-9;
  return out;
}
inline double nominal_acceleration(const Config& c,const ReferenceSample& r,double speed,
                                   double acceleration=0.,bool derivative_valid=false,double steering_angle=0.) {
  auto out=control(c,r,speed,acceleration,derivative_valid,steering_angle);
  // Static nominal response around the scheduled inverse model; not measured
  // body acceleration, slip truth, or an identified motor-delay model.
  // Match the added current with the same assumed turn load. Saturation leaves
  // less current for longitudinal acceleration; compensation is not free thrust.
  return r.acceleration+(out.current-out.feedforward-out.steering)/out.gain;
}
class WheelDerivative {
 public:
  explicit WheelDerivative(Config config=Config{}):cfg_(config){cfg_.validate();}
  void reset(){stamp_=std::numeric_limits<double>::quiet_NaN();speed_=acceleration_=0.;valid_=false;}
  bool observe(double stamp,double speed){
    if(!std::isfinite(stamp)||stamp<=0||!std::isfinite(speed)){reset();return false;}
    if(std::isfinite(stamp_)&&stamp<stamp_){reset();return false;}
    if(stamp==stamp_)return false;
    if(!std::isfinite(stamp_)||stamp-stamp_>cfg_.source_gap+time_epsilon(stamp,stamp_)){
      valid_=false;acceleration_=0.;
    }else{
      const double dt=stamp-stamp_,raw=(speed-speed_)/dt;
      acceleration_=valid_?acceleration_+(1.-std::exp(-dt/cfg_.derivative_tau))*(raw-acceleration_):raw;
      valid_=true;
    }
    stamp_=stamp;speed_=speed;return true;
  }
  bool valid()const{return valid_;}
  double acceleration()const{return acceleration_;}
  double stamp()const{return stamp_;}
 private:
  Config cfg_;double stamp_=std::numeric_limits<double>::quiet_NaN(),speed_=0.,acceleration_=0.;bool valid_=false;
};
} // namespace aims_longitudinal_control
