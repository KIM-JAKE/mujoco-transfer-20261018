// Read-only measured-model IK. This executable never calls control(), stop(),
// automaticErrorRecovery(), or a gripper operation. Output is a candidate only.
#include "servo_core.hpp"
#include "robot_mode.hpp"
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

template <std::size_t N> void array(const std::array<double,N>& a) {
  std::cout << '[';
  for(std::size_t i=0;i<N;++i) {if(i)std::cout<<',';std::cout<<a[i];}
  std::cout<<']';
}
Eigen::Matrix4d read_target(const char* file) {
  std::ifstream stream(file);Eigen::Matrix4d T;
  for(int r=0;r<4;++r)for(int c=0;c<4;++c)
    if(!(stream>>T(r,c)) || !std::isfinite(T(r,c)))throw std::runtime_error("Expected 16 finite row-major target values");
  std::string extra;if(stream>>extra)throw std::runtime_error("Extra target values");
  const auto R=T.block<3,3>(0,0);
  if((T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-9 ||
     (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()>1e-6 || std::abs(R.determinant()-1)>1e-6)
    throw std::runtime_error("Target must be a proper rigid transform in metres");
  if(T.block<3,1>(0,3).norm()>2)throw std::runtime_error("Target translation outside metric robot envelope");
  return T;
}
int main(int argc,char** argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--help") {
      std::cout<<"m80_camera_plan --check-target TARGET.txt\n"
        <<"m80_camera_plan --read-model ROBOT_IP TARGET.txt\n"
        <<"Read-only IK candidate. No collision validation and no motion.\n";return 0;
    }
    if(argc==3 && std::string(argv[1])=="--check-target") {
      read_target(argv[2]);std::cout<<"{\"target_valid\":true,\"robot_connected\":false,\"motion_executed\":false}\n";return 0;
    }
    if(argc!=4 || std::string(argv[1])!="--read-model")throw std::runtime_error("Use --help; an explicit --read-model is required");
    const auto target=read_target(argv[3]); // malformed input rejected before connection
    franka::Robot robot(argv[2],franka::RealtimeConfig::kIgnore);
    if(robot.serverVersion()!=10)throw std::runtime_error("FR3 protocol 10 required");
    const auto s=robot.readOnce();m80::require_idle_mode(s.robot_mode);
    for(std::size_t j=0;j<7;++j)
      if(std::abs(s.dq[j])>.01 || std::abs(s.dq_d[j])>.01 || std::abs(s.q[j]-s.q_d[j])>.005)
        throw std::runtime_error("Robot must be stationary before planning");
    auto model=robot.loadModel();auto q=s.q_d;bool converged=false;
    double position_error=0,angle_error=0;
    for(int iteration=0;iteration<500;++iteration) {
      const auto raw=model.pose(franka::Frame::kEndEffector,q,s.F_T_EE,s.EE_T_K);
      const Eigen::Matrix4d current=Eigen::Map<const Eigen::Matrix4d>(raw.data());
      Eigen::Matrix<double,6,1> e;e.head<3>()=target.block<3,1>(0,3)-current.block<3,1>(0,3);
      const Eigen::AngleAxisd rotation(target.block<3,3>(0,0)*current.block<3,3>(0,0).transpose());
      e.tail<3>()=rotation.axis()*rotation.angle();
      position_error=e.head<3>().norm();angle_error=std::abs(rotation.angle());
      if(position_error<.0001 && angle_error<.001){converged=true;break;}
      const auto jac=model.zeroJacobian(franka::Frame::kEndEffector,q,s.F_T_EE,s.EE_T_K);
      const Eigen::Matrix<double,6,7> J=Eigen::Map<const Eigen::Matrix<double,6,7>>(jac.data());
      const auto step=(J.transpose()*(J*J.transpose()+.0001*Eigen::Matrix<double,6,6>::Identity()).ldlt().solve(e)).eval();
      if(!step.allFinite())throw std::runtime_error("Nonfinite IK step");
      const double scale=std::min(1.,.025/std::max(1e-9,step.cwiseAbs().maxCoeff()));
      for(std::size_t j=0;j<7;++j)q[j]=std::clamp(q[j]+scale*step[j],m80::lower[j]+.001,m80::upper[j]-.001);
    }
    if(!converged)throw std::runtime_error("No bounded measured-model IK solution; no motion sent");
    double duration=2.;bool valid=false;
    for(;duration<=60.;duration+=.5) {
      try {m80::Trajectory({{0.,s.q_d,{},{}},{duration,q,{},{}}}).validate(.1);valid=true;break;}
      catch(const std::exception&) {}
    }
    if(!valid)throw std::runtime_error("No <=60s candidate within 10-percent continuous joint limits");
    std::cout<<std::setprecision(17)<<"{\"read_only\":true,\"motion_executed\":false,\"hardware_ready\":false,\"collision_checked\":false,\"server_protocol_version\":10,\"duration_s\":"<<duration;
    std::cout<<",\"q_start\":";array(s.q_d);std::cout<<",\"q_goal\":";array(q);
    std::cout<<",\"state\":{\"q\":";array(s.q);std::cout<<",\"q_d\":";array(s.q_d);
    std::cout<<",\"dq\":";array(s.dq);std::cout<<",\"dq_d\":";array(s.dq_d);
    std::cout<<",\"F_T_EE_column_major\":";array(s.F_T_EE);
    std::cout<<",\"O_T_EE_column_major\":";array(s.O_T_EE);
    std::cout<<",\"server_protocol_version\":10,\"m_ee_kg\":"<<s.m_ee<<",\"m_load_kg\":"<<s.m_load<<"},\"position_error_m\":"<<position_error<<",\"angle_error_rad\":"<<angle_error<<"}\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<"Read-only camera planning failed: "<<e.what()<<'\n';return 1;}
}
