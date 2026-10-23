// Explicit stopped-boundary payload configuration. No control(), setEE(),
// gripper command, automatic motion, error recovery or tool-parameter writes.
#include "payload_load_core.hpp"
#include "robot_mode.hpp"
#include <franka/robot.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <unistd.h>

namespace {
using m80_payload::Parameters;
Parameters tool(const franka::RobotState& s) {return {s.m_ee,s.F_x_Cee,s.I_ee};}
Parameters load(const franka::RobotState& s) {return {s.m_load,s.F_x_Cload,s.I_load};}
void stationary(const franka::RobotState& s) {
  m80::require_idle_mode(s.robot_mode);
  if(s.current_errors)throw std::runtime_error("Faulted robot cannot change payload");
  for(std::size_t j=0;j<7;++j)
    if(!std::isfinite(s.dq[j])||!std::isfinite(s.dq_d[j])||!std::isfinite(s.q[j])||!std::isfinite(s.q_d[j])||
       std::abs(s.dq[j])>.001||std::abs(s.dq_d[j])>.001||std::abs(s.q[j]-s.q_d[j])>.005)
      throw std::runtime_error("Stationary measured idle boundary required; no payload writes in a control callback");
}
void frame_matches(const franka::RobotState& s,const m80_payload::Request& r) {
  for(std::size_t j=0;j<16;++j)if(!std::isfinite(s.F_T_EE[j])||std::abs(s.F_T_EE[j]-r.F_T_EE[j])>1e-8)
    throw std::runtime_error("Measured TCP changed before payload coordinate conversion could be applied");
}
void ready() {
  std::cout<<"PAYLOAD_READY_REQUIRED\n"<<std::flush;
  pollfd descriptor{STDIN_FILENO,POLLIN,0};
  if(poll(&descriptor,1,1000)<=0||!(descriptor.revents&POLLIN))throw std::runtime_error("Explicit field READY gate absent");
  char bytes[16];const auto count=read(STDIN_FILENO,bytes,sizeof(bytes));
  if(count!=6||std::string(bytes,static_cast<std::size_t>(count))!="READY\n")
    throw std::runtime_error("Payload transition cancelled before configuration write");
}
template<std::size_t N> void array(const std::array<double,N>& a) {
  std::cout<<'[';for(std::size_t j=0;j<N;++j){if(j)std::cout<<',';std::cout<<a[j];}std::cout<<']';
}
}
int main(int argc,char** argv) {
  try {
    const bool validate=argc==3&&std::string(argv[1])=="--validate-request";
    const bool execute=argc==4&&std::string(argv[1])=="--execute-set-load";
    if(!validate&&!execute)throw std::runtime_error("Usage: m80_payload_load --validate-request FILE | --execute-set-load ROBOT_IP FILE");
    std::ifstream stream(argv[validate?2:3]);if(!stream)throw std::runtime_error("Cannot read explicit measured request");
    const auto request=m80_payload::parse(stream);
    if(validate){std::cout<<"{\"request_validated\":true,\"robot_connected\":false,\"hardware_verified\":false}\n";return 0;}
    // kIgnore applies only to this non-realtime configuration operation.
    // This executable never starts a motion controller.
    franka::Robot robot(argv[2],franka::RealtimeConfig::kIgnore);
    if(robot.serverVersion()!=10)throw std::runtime_error("Protocol-10 measured field setup required");
    auto before=robot.readOnce();stationary(before);frame_matches(before,request);
    if(!m80_payload::same(tool(before),request.tool)||!m80_payload::same(load(before),request.before))
      throw std::runtime_error("Configured tool or previous load differs from measured transition; refuse overwrite");
    ready();
    // Re-read after waiting for the field I/O gate. Another application must
    // also be excluded by the caller's per-arm/cell control lock.
    before=robot.readOnce();stationary(before);frame_matches(before,request);
    if(!m80_payload::same(tool(before),request.tool)||!m80_payload::same(load(before),request.before))
      throw std::runtime_error("Robot parameters changed while awaiting READY");
    robot.setLoad(request.after.mass,request.after.center,request.after.inertia);
    bool confirmed=false;franka::RobotState after;
    for(int sample=0;sample<100;++sample) {
      after=robot.readOnce();stationary(after);frame_matches(after,request);
      if(!m80_payload::same(tool(after),request.tool))throw std::runtime_error("Tool parameters changed during payload transition");
      if(m80_payload::same(load(after),request.after)){confirmed=true;break;}
    }
    if(!confirmed)throw std::runtime_error("Payload write not confirmed; ownership state is unknown, inhibit all motion");
    std::cout<<std::setprecision(17)<<"{\"payload_set_and_read_back\":true,\"hardware_verified\":false,\"motion_started\":false,\"tool_unchanged\":true,\"mass_kg\":"<<after.m_load<<",\"F_x_Cload_m\":";
    array(after.F_x_Cload);std::cout<<",\"I_load_column_major_kg_m2\":";array(after.I_load);std::cout<<"}\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<"Payload transition failed; inhibit motion: "<<e.what()<<'\n';return 2;}
}
