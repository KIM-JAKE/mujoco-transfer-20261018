// Native Franka Hand commissioning, using the robot controller IP.
#include "site_watchdog.hpp"
#include <franka/gripper.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted=0;
void interrupt(int) {interrupted=1;}
double number(const char* s) {
  std::size_t n=0;double x=std::stod(s,&n);
  if(s[n] || !std::isfinite(x))throw std::runtime_error("Finite number required");
  return x;
}
void state(const franka::GripperState& s,bool completed=true) {
  if(!std::isfinite(s.width)||!std::isfinite(s.max_width))throw std::runtime_error("Nonfinite gripper state");
  std::cout<<std::setprecision(16)<<"{\"completed\":"<<(completed?"true":"false")
    <<",\"width_m\":"<<s.width<<",\"max_width_m\":"<<s.max_width
    <<",\"is_grasped\":"<<(s.is_grasped?"true":"false")
    <<",\"temperature_c\":"<<s.temperature<<",\"robot_time_s\":"<<s.time.toSec()<<"}\n"<<std::flush;
}
}
int main(int argc,char** argv) {
 try {
  if(argc<3)throw std::runtime_error("Usage: m80_franka_hand status|stop IP; --execute-homing IP; --execute-move IP WIDTH_M SPEED_M_S; --execute-grasp IP WIDTH_M SPEED_M_S FORCE_N EPS_INNER_M EPS_OUTER_M. Actuation requires READY stdin heartbeat.");
  std::string mode=argv[1];
  bool grasp=mode=="--execute-grasp",move=mode=="--execute-move",home=mode=="--execute-homing";
  if(!((mode=="status"||mode=="stop"||home)&&argc==3) && !(move&&argc==5) && !(grasp&&argc==8))throw std::runtime_error("Invalid hand command or argument count");
  double width=0,speed=0,force=0,inner=0,outer=0;
  if(move||grasp) {
    width=number(argv[3]);speed=number(argv[4]);
    if(width<0||width>.08||speed<=0||speed>.02)throw std::runtime_error("Commissioning width 0..0.08m, speed (0,0.02]m/s required");
  }
  if(grasp) {
    force=number(argv[5]);inner=number(argv[6]);outer=number(argv[7]);
    if(force<5||force>20||inner<=0||inner>.002||outer<=0||outer>.002)throw std::runtime_error("Commissioning force 5..20N, epsilon (0,0.002]m required");
  }
  franka::Gripper hand(argv[2]);
  if(mode=="status") {state(hand.readOnce());return 0;}
  if(mode=="stop") {if(!hand.stop())throw std::runtime_error("Hand stop rejected");state(hand.readOnce());return 0;}
  if(!home) {
    auto s=hand.readOnce();
    if(!(s.max_width>0)||width>s.max_width)throw std::runtime_error("Width exceeds homed max_width; explicit homing may be required");
  }
  std::signal(SIGINT,interrupt);std::signal(SIGTERM,interrupt);
  std::atomic<bool> done{false},ready{false},abort{false};
  std::cout<<"HEARTBEAT_REQUIRED\n"<<std::flush;
  std::thread monitor([&]{
    m80::SiteHeartbeat heartbeat(std::chrono::steady_clock::now());std::string buffer;
    while(!done.load()) {
      pollfd p{STDIN_FILENO,POLLIN,0};int rc=poll(&p,1,20);
      if(rc<0)heartbeat.disconnect();
      if(rc>0&&(p.revents&(POLLIN|POLLHUP|POLLERR|POLLNVAL))) {
        char bytes[128];auto n=read(STDIN_FILENO,bytes,sizeof(bytes));
        if(n<=0)heartbeat.disconnect();else {
          buffer.append(bytes,n);std::size_t pos;
          while((pos=buffer.find('\n'))!=std::string::npos){heartbeat.line(buffer.substr(0,pos),std::chrono::steady_clock::now());buffer.erase(0,pos+1);}
          if(buffer.size()>64)heartbeat.disconnect();
        }
      }
      heartbeat.update(std::chrono::steady_clock::now());
      if(interrupted||heartbeat.aborted()) {
        abort=true;ready=false;
        // Gripper members are threadsafe (official libfranka API).
        // Repeat stop until command thread finishes, covering startup races.
        while(!done.load()) {try{hand.stop();}catch(...){}std::this_thread::sleep_for(std::chrono::milliseconds(20));}
        return;
      }
      ready=heartbeat.ready();
    }
  });
  bool ok=false;std::exception_ptr error;
  try {
    while(!ready.load()&&!abort.load())std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if(abort.load())throw std::runtime_error("Hand heartbeat absent or cancelled");
    ok=home?hand.homing():grasp?hand.grasp(width,speed,force,inner,outer):hand.move(width,speed);
    if(abort.load())throw std::runtime_error("Hand action aborted");
    if(!ok)throw std::runtime_error("Hand command failed; no successful action acknowledged");
  }catch(...){error=std::current_exception();}
  done=true;monitor.join();
  if(error)std::rethrow_exception(error);
  if(abort.load())throw std::runtime_error("Hand action interrupted before acknowledgement");
  auto s=hand.readOnce();
  if(grasp&&!s.is_grasped)throw std::runtime_error("No measured object grasp");
  if(move&&(std::abs(s.width-width)>.001||s.is_grasped))throw std::runtime_error("Hand move did not reach free requested width");
  state(s);return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
