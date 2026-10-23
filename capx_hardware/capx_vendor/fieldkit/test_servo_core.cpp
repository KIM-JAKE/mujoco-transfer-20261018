#include "servo_core.hpp"
#include <iostream>
void check(bool condition){if(!condition)throw std::runtime_error("Servo core assertion failed");}

int main() {
  m80::Joints zero{},q{0,-.2,0,-1.5,0,1.5,0},end=q;end[0]=.02;
  m80::Trajectory path({{0.,q,zero,zero},{2.,end,zero,zero}});
  path.validate(.1);
  const auto first=path.evaluate(0.),last=path.evaluate(2.),middle=path.evaluate(1.);
  check(std::abs(first.q[0])<1e-12 && std::abs(last.q[0]-.02)<1e-12);
  check(std::abs(first.dq[0])<1e-12 && std::abs(last.dq[0])<1e-12);
  check(std::abs(middle.q[0]-.01)<1e-12 && middle.dq[0]>0.);
  bool rejected=false;
  try {m80::Trajectory fast({{0.,q,zero,zero},{.01,end,zero,zero}});fast.validate(.95);}
  catch(const std::runtime_error&){rejected=true;}
  check(rejected);
  rejected=false;
  try {auto beyond=end;beyond[0]=3.;m80::Trajectory bad({{0.,q,zero,zero},{2.,beyond,zero,zero}});bad.validate(.95);}
  catch(const std::runtime_error&){rejected=true;}
  check(rejected);
  rejected=false;
  try {auto velocity=zero;velocity[1]=.1;m80::Trajectory bad({{0.,q,velocity,zero},{2.,end,zero,zero}});}
  catch(const std::runtime_error&){rejected=true;}
  check(rejected);
  std::cout << "PASS: C2 endpoints, Bernstein continuous envelopes, excessive speed/position and moving-start rejection\n";
}
