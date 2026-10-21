// Small, single-arm commissioning moves. All device I/O remains outside RT.
#include "libfranka_stage.hpp"
#include "robot_mode.hpp"
#include "site_watchdog.hpp"
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted=0;
void signal_stop(int) {interrupted=1;}
double number(const char* text) {
  std::size_t end=0;double v=std::stod(text,&end);
  if(text[end]!=0 || !std::isfinite(v))throw std::runtime_error("Finite numeric argument required");
  return v;
}
void record(const char* filename) {
  std::ifstream f(filename);if(!f || f.peek()==std::ifstream::traits_type::eof())
    throw std::runtime_error("Nonempty field record required");
}
m80::Trajectory load_csv(const char* filename) {
  std::ifstream f(filename);if(!f)throw std::runtime_error("Cannot open field trajectory");
  std::string line;std::vector<m80::Waypoint> points;
  while(std::getline(f,line)) {
    if(line.empty() || line[0]=='#')continue;
    if(points.empty() && line.rfind("time_s,",0)==0)continue;
    std::replace(line.begin(),line.end(),',',' ');std::istringstream stream(line);m80::Waypoint p{};
    if(!(stream>>p.time_s) || !std::isfinite(p.time_s))throw std::runtime_error("Invalid CSV time");
    for(auto* a:{&p.q,&p.dq,&p.ddq})for(double& v:*a)
      if(!(stream>>v) || !std::isfinite(v))throw std::runtime_error("Expected 22 finite CSV columns");
    std::string extra;if(stream>>extra)throw std::runtime_error("Extra CSV column");
    points.push_back(p);if(points.size()>100000)throw std::runtime_error("Too many waypoints");
  }
  return m80::Trajectory(points);
}
m80::Joints cartesian_target(franka::Model& model,const franka::RobotState& state,const Eigen::Vector3d& delta) {
  m80::Joints q=state.q_d;
  auto raw=model.pose(franka::Frame::kEndEffector,q,state.F_T_EE,state.EE_T_K);
  Eigen::Matrix4d target=Eigen::Map<const Eigen::Matrix4d>(raw.data());target.block<3,1>(0,3)+=delta;
  for(int iteration=0;iteration<100;++iteration) {
    raw=model.pose(franka::Frame::kEndEffector,q,state.F_T_EE,state.EE_T_K);
    Eigen::Matrix4d current=Eigen::Map<const Eigen::Matrix4d>(raw.data());
    Eigen::Matrix<double,6,1> error;error.head<3>()=target.block<3,1>(0,3)-current.block<3,1>(0,3);
    error.tail<3>().setZero();for(int k=0;k<3;++k)error.tail<3>()+=.5*current.block<3,1>(0,k).cross(target.block<3,1>(0,k));
    if(error.head<3>().norm()<1e-7 && error.tail<3>().norm()<1e-6) {
      for(std::size_t j=0;j<7;++j)if(std::abs(q[j]-state.q_d[j])>.05)throw std::runtime_error("Cartesian jog exceeds small-joint envelope");
      return q;
    }
    const auto jac=model.zeroJacobian(franka::Frame::kEndEffector,q,state.F_T_EE,state.EE_T_K);
    Eigen::Matrix<double,6,7> J=Eigen::Map<const Eigen::Matrix<double,6,7>>(jac.data());
    Eigen::Matrix<double,7,1> step=J.transpose()*(J*J.transpose()+.0001*Eigen::Matrix<double,6,6>::Identity()).ldlt().solve(error);
    if(!step.allFinite())throw std::runtime_error("Invalid IK correction");
    const double scale=std::min(1.,.01/std::max(.000001,step.cwiseAbs().maxCoeff()));
    for(std::size_t j=0;j<7;++j) {
      q[j]+=scale*step[j];if(q[j]<=m80::lower[j] || q[j]>=m80::upper[j])throw std::runtime_error("IK joint envelope exceeded");
    }
  }
  throw std::runtime_error("Small Cartesian jog IK did not converge");
}
void model_check(franka::Model& model,const franka::RobotState& s,const m80::Trajectory& path) {
  m80::Joints previous{};bool first=true;
  for(double t=0.;t<=path.duration()+.0001;t+=.001) {
    const auto p=path.evaluate(std::min(t,path.duration()));
    const auto M=model.mass(p.q,s.I_total,s.m_total,s.F_x_Ctotal);
    const auto C=model.coriolis(p.q,p.dq,s.I_total,s.m_total,s.F_x_Ctotal,s.O_ddP_O);
    const auto G=model.gravity(p.q,s.m_total,s.F_x_Ctotal,s.O_ddP_O);
    m80::Joints torque{};
    for(std::size_t j=0;j<7;++j) {
      torque[j]=C[j];for(std::size_t k=0;k<7;++k)torque[j]+=M[j+7*k]*p.ddq[k];
      if(!std::isfinite(torque[j]) || !std::isfinite(G[j]) || std::abs(torque[j]+G[j])>(j<4?87.:12.) ||
         (!first && std::abs(torque[j]-previous[j])/.001>1000.))
        throw std::runtime_error("Actual SDK model torque/rate screen failed");
    }
    previous=torque;first=false;
  }
}
}

int main(int argc,char** argv) {
  // Common arguments: mode IP tool_mass load_mass calibration_record commissioning_record log
  // Jog tail: joint_index delta_rad duration; Cartesian tail: dx dy dz duration; CSV tail: file
  try {
    if(argc==3 && std::string(argv[1])=="--validate-csv") {
      load_csv(argv[2]).validate(.1);
      std::cout<<"{\"continuous_10percent_pass\":true,\"hardware_ready\":false,\"robot_connected\":false}\n";return 0;
    }
    if(argc<9)throw std::runtime_error("Usage: m80_site_motion --execute-jog|--execute-cartesian|--execute-csv IP TOOL_KG LOAD_KG CALIBRATION_RECORD COMMISSIONING_RECORD TELEMETRY.csv [TARGET]");
    const std::string mode=argv[1];
    const bool tracking_option=argc>=11 && std::string(argv[argc-2])=="--tracking-error-rad";
    const int execution_argc=argc-(tracking_option?2:0);
    if(tracking_option && mode!="--execute-csv")throw std::runtime_error("Explicit tracking option is supported only for CSV stages");
    if(!((mode=="--execute-jog" && execution_argc==11) || (mode=="--execute-cartesian" && execution_argc==12) || (mode=="--execute-csv" && execution_argc==9)))
      throw std::runtime_error("Invalid explicit execution mode or argument count");
    m80::FieldContext site;site.command_fraction=.1;site.expected_tool_mass_kg=number(argv[3]);site.expected_load_mass_kg=number(argv[4]);
    if(tracking_option) {
      site.maximum_tracking_error_rad=number(argv[argc-1]);
      if(site.maximum_tracking_error_rad<.0001 || site.maximum_tracking_error_rad>.02)
        throw std::runtime_error("Tracking error guard must be within 0.0001..0.02rad");
    }
    if(site.expected_tool_mass_kg<0 || site.expected_load_mass_kg<0)throw std::runtime_error("Payload masses must be nonnegative");
    record(argv[5]);record(argv[6]);site.calibration_record=argv[5];site.commissioning_record=argv[6];
    site.field_planned=site.tool_verified=site.io_verified=true;
    site.maximum_external_force_n=5.;site.maximum_external_torque_nm=1.;
    // Validate syntax and movement bounds before even opening a robot connection.
    double duration=0.,joint=0.,delta=0.;Eigen::Vector3d xyz=Eigen::Vector3d::Zero();
    if(mode=="--execute-jog") {
      joint=number(argv[8]);delta=number(argv[9]);duration=number(argv[10]);
      if(joint!=std::floor(joint) || joint<1 || joint>7 || std::abs(delta)>.05 || delta==0)throw std::runtime_error("Jog needs joint 1..7 and nonzero delta <=0.05rad");
    } else if(mode=="--execute-cartesian") {
      xyz=Eigen::Vector3d(number(argv[8]),number(argv[9]),number(argv[10]));duration=number(argv[11]);
      if(xyz.norm()>.005 || xyz.norm()==0)throw std::runtime_error("Cartesian jog needs nonzero displacement <=5mm in base frame");
    }
    if(mode!="--execute-csv" && !(duration>=2. && duration<=30.))throw std::runtime_error("Jog duration must be 2..30 seconds");
    std::ofstream log(argv[7]);if(!log)throw std::runtime_error("Cannot open telemetry output");
    log<<"index,t_s,robot_s,period_s";
    for(const auto& name:{"q","dq","q_d","dq_d","ddq_d","tau_J","tau_J_d","tau_ext_hat_filtered"})for(int j=1;j<=7;++j)log<<','<<name<<j;
    for(int j=0;j<6;++j)log<<",O_F_ext_hat_K"<<j;
    for(int j=0;j<16;++j)log<<",O_T_EE"<<j;
    log<<",robot_mode,current_errors,io_ready,abort_requested\n";
    franka::Robot robot(argv[2]);
    if(robot.serverVersion()!=10)throw std::runtime_error("FR3 protocol 10 required");
    const auto state=robot.readOnce();m80::require_idle_mode(state.robot_mode);
    auto model=robot.loadModel();m80::Joints end=state.q_d;
    if(mode=="--execute-jog")end[static_cast<int>(joint)-1]+=delta;
    if(mode=="--execute-cartesian")end=cartesian_target(model,state,xyz);
    m80::Trajectory path=mode=="--execute-csv"?load_csv(argv[8]):m80::Trajectory({{0.,state.q_d,{},{}},{duration,end,{},{}}});
    path.validate(.1);
    // Field CSV stages deliberately remain a commissioning tool at <=10%.
    model_check(model,state,path);site.torque_budget_verified=true;m80::validate_field_context(site);
    std::cout<<"HEARTBEAT_REQUIRED\n"<<std::flush;
    m80::AtomicGates gates;m80::StageTelemetry telemetry;site.telemetry=&telemetry;
    std::atomic<bool> done{false};std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
    std::thread monitor([&] {
      m80::SiteHeartbeat heartbeat(std::chrono::steady_clock::now());std::string buffer;
      while(!done.load()) {
        pollfd p{STDIN_FILENO,POLLIN,0};const int result=poll(&p,1,20);
        if(result<0)heartbeat.disconnect();
        if(result>0 && (p.revents&(POLLIN|POLLHUP|POLLERR|POLLNVAL))) {
          char bytes[128];const auto count=read(STDIN_FILENO,bytes,sizeof(bytes));
          if(count<=0)heartbeat.disconnect();
          else {
            buffer.append(bytes,count);std::size_t pos;
            while((pos=buffer.find('\n'))!=std::string::npos) {
              const auto line=buffer.substr(0,pos);buffer.erase(0,pos+1);
              heartbeat.line(line,std::chrono::steady_clock::now());
              if(line=="STOP")std::cout<<"EXPLICIT_STOP_ACCEPTED\n"<<std::flush;
            }
            if(buffer.size()>64)heartbeat.disconnect();
          }
        }
        heartbeat.update(std::chrono::steady_clock::now());
        if(interrupted || heartbeat.aborted())gates.abort_requested=true;
        gates.io_ready=heartbeat.ready() && !gates.abort_requested.load();
        if(gates.abort_requested.load()) {
          gates.io_ready=false;
          try {robot.stop();std::cout<<"SDK_STOP_RETURNED\n"<<std::flush;}catch(...) {}
          break;
        }
      }
    });
    std::thread writer([&] {
      m80::TelemetrySample s;log<<std::setprecision(17);
      while(true) {
        if(!telemetry.try_pop(s)) {
          if(done.load())break;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;
        }
        log<<s.sample_index<<','<<s.elapsed_s<<','<<s.robot_time_s<<','<<s.period_s;
        for(const auto* a:{&s.q,&s.dq,&s.q_d,&s.dq_d,&s.ddq_d,&s.tau_J,&s.tau_J_d,&s.tau_ext_hat_filtered})for(double v:*a)log<<','<<v;
        for(double v:s.O_F_ext_hat_K)log<<','<<v;
        for(double v:s.O_T_EE)log<<','<<v;
        log<<','<<s.robot_mode<<','<<s.current_errors<<','<<s.io_ready<<','<<s.abort_requested<<'\n';
        if(s.sample_index%50==0)log.flush();
        if(!log)gates.abort_requested=true;
      }
    });
    try {
      for(int k=0;k<100 && !gates.io_ready.load() && !gates.abort_requested.load();++k)std::this_thread::sleep_for(std::chrono::milliseconds(5));
      const auto report=m80::run_joint_stage(robot,path,site,gates);
      done=true;monitor.join();writer.join();log.flush();
      if(!log)throw std::runtime_error("Telemetry write failed");
      std::cout<<"{\"stage_completed\":true,\"hardware_ready\":false,\"samples\":"<<report.samples<<",\"telemetry_dropped\":"<<telemetry.dropped_samples()<<"}\n";
    }catch(...) {gates.abort_requested=true;done=true;if(monitor.joinable())monitor.join();if(writer.joinable())writer.join();throw;}
    return 0;
  }catch(const std::exception& error) {std::cerr<<"Commissioning stage failed: "<<error.what()<<'\n';return 1;}
}
