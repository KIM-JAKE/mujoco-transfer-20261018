// A read-only measured FR3 model service. No control(), gripper, or recovery.
#include "servo_core.hpp"
#include "robot_mode.hpp"
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using Matrix=Eigen::Matrix4d;
template<std::size_t N> void array(const std::array<double,N>& a) {
  std::cout<<'[';for(std::size_t i=0;i<N;++i){if(i)std::cout<<',';std::cout<<a[i];}std::cout<<']';
}
void matrix(const Matrix& T) {
  std::cout<<'[';for(int r=0;r<4;++r){if(r)std::cout<<',';std::cout<<'[';for(int c=0;c<4;++c){if(c)std::cout<<',';std::cout<<T(r,c);}std::cout<<']';}std::cout<<']';
}
void validate_target(const Matrix& T) {
  const auto R=T.block<3,3>(0,0);
  if(!T.allFinite() || (T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-9 ||
    (R.transpose()*R-Matrix::Identity().block<3,3>(0,0)).norm()>1e-6 || std::abs(R.determinant()-1)>1e-6 || T.block<3,1>(0,3).norm()>2)
    throw std::runtime_error("Target must be a proper metric base-frame transform inside 2m");
}
std::vector<Matrix> targets(const char* filename) {
  std::ifstream f(filename);if(!f)throw std::runtime_error("Cannot read target file");
  std::string line;std::vector<Matrix> result;
  while(std::getline(f,line)) {
    if(line.empty() || line[0]=='#')continue;
    if(result.size()>=32)throw std::runtime_error("At most 32 target frames per bounded phase");
    std::istringstream s(line);Matrix T;
    for(int r=0;r<4;++r)for(int c=0;c<4;++c)if(!(s>>T(r,c)))throw std::runtime_error("Expected 16 row-major target values per line");
    std::string extra;if(s>>extra)throw std::runtime_error("Extra target columns");
    validate_target(T);result.push_back(T);
  }
  if(result.empty())throw std::runtime_error("No target frames");return result;
}
Matrix pose(franka::Model& model,const franka::RobotState& s,const m80::Joints& q,franka::Frame frame) {
  const auto raw=model.pose(frame,q,s.F_T_EE,s.EE_T_K);
  return Eigen::Map<const Matrix>(raw.data());
}
using Points=std::array<Eigen::Vector3d,9>;
Points points(franka::Model& m,const franka::RobotState& s,const m80::Joints& q) {
  Points p;p[0]=Eigen::Vector3d::Zero();
  const std::array<franka::Frame,7> frames{franka::Frame::kJoint1,franka::Frame::kJoint2,franka::Frame::kJoint3,
    franka::Frame::kJoint4,franka::Frame::kJoint5,franka::Frame::kJoint6,franka::Frame::kJoint7};
  for(std::size_t j=0;j<7;++j)p[j+1]=pose(m,s,q,frames[j]).block<3,1>(0,3);
  p[8]=pose(m,s,q,franka::Frame::kEndEffector).block<3,1>(0,3);return p;
}
void points_json(const Points& p) {
  std::cout<<'[';for(std::size_t i=0;i<p.size();++i){if(i)std::cout<<',';std::cout<<'['<<p[i][0]<<','<<p[i][1]<<','<<p[i][2]<<']';}std::cout<<']';
}
void state_json(const franka::RobotState& s) {
  std::cout<<"{\"read_only\":true,\"server_protocol_version\":10,\"q\":";array(s.q);
  std::cout<<",\"q_d\":";array(s.q_d);std::cout<<",\"dq\":";array(s.dq);std::cout<<",\"dq_d\":";array(s.dq_d);
  std::cout<<",\"F_T_EE_column_major\":";array(s.F_T_EE);std::cout<<",\"O_T_EE_column_major\":";array(s.O_T_EE);
  std::cout<<",\"m_ee_kg\":"<<s.m_ee<<",\"m_load_kg\":"<<s.m_load<<'}';
}
m80::Joints solve(franka::Model& m,const franka::RobotState& s,const Matrix& target,m80::Joints q) {
  for(int iteration=0;iteration<700;++iteration) {
    const auto T=pose(m,s,q,franka::Frame::kEndEffector);
    Eigen::Matrix<double,6,1> error;error.head<3>()=target.block<3,1>(0,3)-T.block<3,1>(0,3);
    const Eigen::AngleAxisd a(target.block<3,3>(0,0)*T.block<3,3>(0,0).transpose());error.tail<3>()=a.axis()*a.angle();
    if(error.head<3>().norm()<.00005 && std::abs(a.angle())<.0005)return q;
    const auto raw=m.zeroJacobian(franka::Frame::kEndEffector,q,s.F_T_EE,s.EE_T_K);
    const Eigen::Matrix<double,6,7> J=Eigen::Map<const Eigen::Matrix<double,6,7>>(raw.data());
    const auto step=(J.transpose()*(J*J.transpose()+.00005*Eigen::Matrix<double,6,6>::Identity()).ldlt().solve(error)).eval();
    if(!step.allFinite())throw std::runtime_error("Nonfinite IK step");
    const double scale=std::min(1.,.025/std::max(1e-9,step.cwiseAbs().maxCoeff()));
    for(std::size_t j=0;j<7;++j)q[j]=std::clamp(q[j]+scale*step[j],m80::lower[j]+.001,m80::upper[j]-.001);
  }
  throw std::runtime_error("No measured-model bounded IK for this phase");
}
int main(int argc,char** argv) {
  try {
    const std::string mode=argc>1?argv[1]:"";
    if(argc==2 && mode=="--help") {
      std::cout<<"m80_dynamic_plan --check-targets TARGETS.txt\n"
       <<"m80_dynamic_plan --read-state ROBOT_IP\n"
       <<"m80_dynamic_plan --read-model ROBOT_IP TARGETS.txt\n"
       <<"READ ONLY. Native C2 samples, SDK FK, and velocity bounds; no collision or hardware readiness claim.\n";return 0;
    }
    if(argc==3 && mode=="--check-targets") {
      const auto ts=targets(argv[2]);std::cout<<"{\"targets_valid\":true,\"count\":"<<ts.size()<<",\"robot_connected\":false}\n";return 0;
    }
    if(!((argc==3 && mode=="--read-state") || (argc==4 && mode=="--read-model")))throw std::runtime_error("Explicit --read-state or --read-model required; see --help");
    const auto ts=mode=="--read-model"?targets(argv[3]):std::vector<Matrix>{};
    franka::Robot robot(argv[2],franka::RealtimeConfig::kIgnore);
    if(robot.serverVersion()!=10)throw std::runtime_error("FR3 protocol 10 required");
    const auto s=robot.readOnce();m80::require_idle_mode(s.robot_mode);
    for(std::size_t j=0;j<7;++j)
      if(std::abs(s.dq[j])>.01 || std::abs(s.dq_d[j])>.01 || std::abs(s.q[j]-s.q_d[j])>.005)
        throw std::runtime_error("Stationary measured state required for planning");
    auto model=robot.loadModel();const auto initial_points=points(model,s,s.q_d);
    std::cout<<std::setprecision(17);
    if(mode=="--read-state") {
      std::cout<<"{\"read_only\":true,\"motion_executed\":false,\"server_protocol_version\":10,\"state\":";state_json(s);
      std::cout<<",\"link_points_base_m\":";points_json(initial_points);
      std::cout<<",\"tcp_base\":";matrix(pose(model,s,s.q_d,franka::Frame::kEndEffector));std::cout<<"}\n";return 0;
    }
    std::vector<m80::Waypoint> knots{{0.,s.q_d,{},{}}};auto q=s.q_d;
    for(const auto& T:ts) {
      const auto goal=solve(model,s,T,q);double h=2.;bool valid=false;
      for(;h<=60.;h+=.5) {
        try {m80::Trajectory({{0.,q,{},{}},{h,goal,{},{}}}).validate(.1);valid=true;break;}
        catch(const std::exception&) {}
      }
      if(!valid || knots.back().time_s+h>60.)throw std::runtime_error("Phase exceeds 60s continuous 10-percent envelope");
      knots.push_back({knots.back().time_s+h,goal,{},{}});q=goal;
    }
    m80::Trajectory path(knots);path.validate(.1);
    m80::Joints velocities{};
    for(std::size_t k=1;k<knots.size();++k)for(std::size_t j=0;j<7;++j)
      velocities[j]=std::max(velocities[j],1.875*std::abs(knots[k].q[j]-knots[k-1].q[j])/(knots[k].time_s-knots[k-1].time_s));
    std::array<double,8> lengths{};std::array<double,9> speeds{};
    for(std::size_t i=0;i<8;++i)lengths[i]=(initial_points[i+1]-initial_points[i]).norm()+1e-6;
    for(std::size_t k=1;k<9;++k)for(std::size_t j=0;j<7 && j+1<k;++j) {
      double reach=0;for(std::size_t r=j+1;r<k;++r)reach+=lengths[r];speeds[k]+=reach*velocities[j];
    }
    // Validate all SDK geometry first; never emit a half-valid success record.
    for(double t=0.;t<=path.duration()+1e-8;t+=.001) {
      const auto ps=points(model,s,path.evaluate(std::min(t,path.duration())).q);
      for(std::size_t i=0;i<8;++i)if((ps[i+1]-ps[i]).norm()>lengths[i]+1e-7)
        throw std::runtime_error("SDK consecutive joint-frame lengths are not rigid; cannot certify sweep bound");
    }
    std::cout<<"{\"read_only\":true,\"motion_executed\":false,\"hardware_ready\":false,\"collision_checked\":false,\"server_protocol_version\":10,\"continuous_10percent_checked\":true,\"state\":";
    state_json(s);std::cout<<",\"q_start\":";array(s.q_d);
    std::cout<<",\"duration_s\":"<<path.duration()<<",\"sample_dt_s\":0.001,\"joint_velocity_bound_rad_s\":";array(velocities);
    std::cout<<",\"link_segment_lengths_m\":";array(lengths);
    std::cout<<",\"link_point_speed_bounds_m_s\":";array(speeds);
    std::cout<<",\"knots\":[";
    for(std::size_t i=0;i<knots.size();++i){if(i)std::cout<<',';std::cout<<"{\"time_s\":"<<knots[i].time_s<<",\"q\":";array(knots[i].q);std::cout<<",\"dq\":";array(knots[i].dq);std::cout<<",\"ddq\":";array(knots[i].ddq);std::cout<<'}';}
    std::cout<<"],\"samples\":[";const auto count=static_cast<std::size_t>(std::ceil(path.duration()/.001));
    for(std::size_t i=0;i<=count;++i) {
      const double t=std::min(i*.001,path.duration());const auto cmd=path.evaluate(t);if(i)std::cout<<',';
      std::cout<<"{\"time_s\":"<<t<<",\"q\":";array(cmd.q);std::cout<<",\"dq\":";array(cmd.dq);
      std::cout<<",\"tcp_base\":";matrix(pose(model,s,cmd.q,franka::Frame::kEndEffector));
      std::cout<<",\"link_points_base_m\":";points_json(points(model,s,cmd.q));std::cout<<'}';
    }
    std::cout<<"]}\n";return 0;
  }catch(const std::exception& e){std::cerr<<"Read-only dynamic model failed: "<<e.what()<<'\n';return 1;}
}
