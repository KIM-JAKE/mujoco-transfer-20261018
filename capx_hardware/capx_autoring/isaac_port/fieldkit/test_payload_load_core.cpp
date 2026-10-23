#include "payload_load_core.hpp"
#include <sstream>
#include <iostream>

int main() {
  using namespace m80_payload;
  const Parameters empty{0.,{0.,0.,0.},{0.,0.,0.,0.,0.,0.,0.,0.,0.}};
  const Parameters positive{.1,{.01,.02,.03},{1e-5,0.,0.,0.,2e-5,0.,0.,0.,2e-5}};
  auto fail=[](const Parameters& p) {try{validate(p);}catch(const std::exception&){return true;}return false;};
  validate(empty);validate(positive);
  auto p=positive;p.inertia={};if(!fail(p))return 1;
  p=positive;p.mass=-.1;if(!fail(p))return 2;
  p=positive;p.inertia[1]=.1;if(!fail(p))return 3;
  p=positive;p.inertia[8]=.01;if(!fail(p))return 4;
  p=empty;p.center[0]=.1;if(!fail(p))return 5;
  if(!same(positive,positive)||same(positive,empty))return 6;
  auto text=[](const Parameters& tool,const Parameters& before,const Parameters& after) {
    std::ostringstream o;o<<"1\n";
    for(auto* p:{&tool,&before,&after}) {o<<p->mass;for(double x:p->center)o<<' '<<x;for(double x:p->inertia)o<<' '<<x;o<<'\n';}
    o<<"1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n";return o.str();
  };
  auto tool=positive;tool.mass=.73;std::istringstream good(text(tool,empty,positive));parse(good);
  p=positive;p.mass=2.5;bool rejected=false;
  try{std::istringstream bad(text(tool,empty,p));parse(bad);}catch(...){rejected=true;}
  if(!rejected)return 7;
  std::cout<<"Payload validation offline passed; no hardware connected\n";
}
