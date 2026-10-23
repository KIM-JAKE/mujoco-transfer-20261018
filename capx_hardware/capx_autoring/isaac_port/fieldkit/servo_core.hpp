#pragma once
// Pure C++ trajectory core. No robot connection, ROS or Python dependency.
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace m80 {
using Joints = std::array<double, 7>;
inline constexpr Joints lower{-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,.4398,-3.0508};
inline constexpr Joints upper{2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508};
inline constexpr Joints vmax{2.62,2.62,2.62,2.62,5.26,4.18,5.26};
inline constexpr Joints offset{.6599,.2517,.2,.3533,.5757,.4878,.4628};
inline constexpr Joints decel{6.,2.585,3.5,4.,17.,5.5,17.};
struct Waypoint { double time_s; Joints q, dq, ddq; };
struct Command { Joints q{}, dq{}, ddq{}, jerk{}; };
using Polygon = std::array<Joints, 6>;

template <std::size_t N>
Joints casteljau(std::array<Joints,N> points, double u) {
  for (std::size_t n=N; n>1; --n)
    for (std::size_t k=0; k<n-1; ++k)
      for (std::size_t j=0; j<7; ++j)
        points[k][j]=(1-u)*points[k][j]+u*points[k+1][j];
  return points[0];
}

struct Segment {
  double start_s, duration_s;
  Polygon q;
  static Segment from(const Waypoint& a, const Waypoint& b) {
    double h=b.time_s-a.time_s;
    if (!(h>0) || !std::isfinite(h)) throw std::runtime_error("Invalid waypoint times");
    Segment segment{a.time_s,h,{}};
    for (std::size_t j=0;j<7;++j) {
      segment.q[0][j]=a.q[j];
      segment.q[1][j]=a.q[j]+h*a.dq[j]/5;
      segment.q[2][j]=a.q[j]+2*h*a.dq[j]/5+h*h*a.ddq[j]/20;
      segment.q[3][j]=b.q[j]-2*h*b.dq[j]/5+h*h*b.ddq[j]/20;
      segment.q[4][j]=b.q[j]-h*b.dq[j]/5;
      segment.q[5][j]=b.q[j];
      for (const auto& point:segment.q)
        if (!std::isfinite(point[j])) throw std::runtime_error("Nonfinite trajectory");
    }
    return segment;
  }
  Command evaluate(double time_s) const {
    const double u=std::clamp((time_s-start_s)/duration_s,0.,1.);
    std::array<Joints,5> v{}; std::array<Joints,4> a{}; std::array<Joints,3> j{};
    for (std::size_t k=0;k<5;++k) for(std::size_t n=0;n<7;++n)
      v[k][n]=5*(q[k+1][n]-q[k][n])/duration_s;
    for (std::size_t k=0;k<4;++k) for(std::size_t n=0;n<7;++n)
      a[k][n]=4*(v[k+1][n]-v[k][n])/duration_s;
    for (std::size_t k=0;k<3;++k) for(std::size_t n=0;n<7;++n)
      j[k][n]=3*(a[k+1][n]-a[k][n])/duration_s;
    return {casteljau(q,u),casteljau(v,u),casteljau(a,u),casteljau(j,u)};
  }
};

inline void validate_polygon(const Polygon& points,double duration,double fraction,int depth=4) {
  // Subdivision tightens the Bernstein convex-hull enclosure without sampling.
  if(depth>0) {
    Polygon work=points,left{},right{};left[0]=work[0];right[5]=work[5];
    for(std::size_t level=1;level<6;++level) {
      for(std::size_t k=0;k<6-level;++k)for(std::size_t j=0;j<7;++j)
        work[k][j]=.5*(work[k][j]+work[k+1][j]);
      left[level]=work[0];right[5-level]=work[5-level];
    }
    validate_polygon(left,duration/2,fraction,depth-1);
    validate_polygon(right,duration/2,fraction,depth-1);
    return;
  }
  for(std::size_t j=0;j<7;++j) {
    double lo=points[0][j],hi=lo;
    for(const auto& p:points){lo=std::min(lo,p[j]);hi=std::max(hi,p[j]);}
    if(lo<lower[j] || hi>upper[j])throw std::runtime_error("Joint position envelope exceeded");
    const double vl=fraction*std::max(-vmax[j],std::min(0.,offset[j]-std::sqrt(std::max(0.,2*decel[j]*(lo-lower[j])))));
    const double vu=fraction*std::min(vmax[j],std::max(0.,-offset[j]+std::sqrt(std::max(0.,2*decel[j]*(upper[j]-hi)))));
    std::array<double,5> v{};std::array<double,4>a{};std::array<double,3>jerk{};
    for(std::size_t k=0;k<5;++k) {
      v[k]=5*(points[k+1][j]-points[k][j])/duration;
      if(v[k]<vl-1e-8 || v[k]>vu+1e-8)throw std::runtime_error("Position-dependent velocity envelope exceeded");
    }
    for(std::size_t k=0;k<4;++k) {
      a[k]=4*(v[k+1]-v[k])/duration;
      if(std::abs(a[k])>10*fraction+1e-7)throw std::runtime_error("Joint acceleration envelope exceeded");
    }
    for(std::size_t k=0;k<3;++k) {
      jerk[k]=3*(a[k+1]-a[k])/duration;
      if(std::abs(jerk[k])>5000*fraction+1e-5)throw std::runtime_error("Joint jerk envelope exceeded");
    }
  }
}

class Trajectory {
 public:
  explicit Trajectory(const std::vector<Waypoint>& points) {
    if(points.size()<2 || points.size()>100000 || points.front().time_s!=0.)
      throw std::runtime_error("Trajectory must start at zero and contain bounded waypoint data");
    for(const auto& endpoint:{points.front(),points.back()})
      for(std::size_t j=0;j<7;++j)
        if(std::abs(endpoint.dq[j])>1e-8 || std::abs(endpoint.ddq[j])>1e-8)
          throw std::runtime_error("Stage endpoints must be stationary with zero acceleration");
    for(std::size_t i=1;i<points.size();++i)segments_.push_back(Segment::from(points[i-1],points[i]));
    duration_=points.back().time_s;
    if(!(duration_>0 && duration_<=60.))throw std::runtime_error("Invalid stage duration");
  }
  void validate(double fraction)const {
    if(!(fraction>0 && fraction<=.95))throw std::runtime_error("Motion fraction must be in (0, 0.95]");
    for(const auto& s:segments_)validate_polygon(s.q,s.duration_s,fraction);
  }
  Command evaluate(double time_s)const {
    auto it=std::upper_bound(segments_.begin(),segments_.end(),time_s,
        [](double time,const Segment& s){return time<s.start_s;});
    if(it!=segments_.begin())--it;
    return it->evaluate(time_s);
  }
  double duration()const{return duration_;}
 private:
  std::vector<Segment>segments_;
  double duration_{};
};
} // namespace m80
