// Read-only SDK inverse-dynamics screen. Never calls control() or config writes.
#include "servo_core.hpp"
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
std::vector<m80::Waypoint> load(const std::string& path) {
  std::ifstream input(path);
  if(!input)throw std::runtime_error("Cannot open trajectory waypoint CSV");
  std::string line;std::vector<m80::Waypoint> result;
  while(std::getline(input,line)) {
    if(line.empty() || line.front()=='#')continue;
    if(result.empty() && line.rfind("time_s,",0)==0)continue;
    std::replace(line.begin(),line.end(),',',' ');std::istringstream stream(line);
    m80::Waypoint point{};
    if(!(stream>>point.time_s))throw std::runtime_error("Invalid CSV time");
    for(auto* values:{&point.q,&point.dq,&point.ddq})for(double& value:*values)
      if(!(stream>>value) || !std::isfinite(value))throw std::runtime_error("CSV needs 22 finite columns: t,q[7],dq[7],ddq[7]");
    std::string extra;if(stream>>extra)throw std::runtime_error("Unexpected extra CSV column");
    if(!std::isfinite(point.time_s) || (!result.empty() && point.time_s<=result.back().time_s))
      throw std::runtime_error("Waypoint times must strictly increase");
    result.push_back(point);
    if(result.size()>100000)throw std::runtime_error("CSV exceeds waypoint budget");
  }
  if(result.size()<2)throw std::runtime_error("Insufficient CSV samples");
  // Check the continuous C2 interpolant, rather than trusting supplied dq/ddq.
  m80::Trajectory(result).validate(.95);
  return result;
}
template<std::size_t N>void array(const std::array<double,N>& values) {
  std::cout<<'[';
  for(std::size_t j=0;j<N;++j){if(j)std::cout<<',';std::cout<<values[j];}
  std::cout<<']';
}
template<std::size_t N>void finite(const std::array<double,N>& values) {
  for(double value:values)if(!std::isfinite(value))throw std::runtime_error("Nonfinite SDK model input/output");
}
} // namespace

int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"Usage: m80_model_audit ROBOT_IP|--validate-only WAYPOINTS.csv\n";return 2;}
  try {
    const auto points=load(argv[2]); // File validation before opening any robot connection.
    if(std::string(argv[1])=="--validate-only") {
      std::cout<<"{\"read_only\":true,\"robot_connected\":false,\"motion_executed\":false,\"hardware_ready\":false,\"continuous_kinematic_screen_pass\":true,\"fraction\":0.95,\"waypoints\":"<<points.size()<<",\"duration_s\":"<<points.back().time_s<<"}\n";
      return 0;
    }
    franka::Robot robot(argv[1],franka::RealtimeConfig::kIgnore);
    if(robot.serverVersion()!=10)throw std::runtime_error("FR3 protocol 10 required for this motion envelope");
    const auto state=robot.readOnce();auto model=robot.loadModel();
    finite(state.I_total);finite(state.F_x_Ctotal);finite(state.O_ddP_O);
    if(!std::isfinite(state.m_total) || state.m_total<0)throw std::runtime_error("Invalid reported payload mass");
    m80::Joints max_total{},max_dynamic{},max_rate{},previous{};
    bool passed=true,first=true;
    const m80::Trajectory trajectory(points);
    const auto samples=static_cast<unsigned long>(std::floor(trajectory.duration()*1000))+1;
    for(unsigned long sample=0;sample<samples;++sample) {
      const auto point=trajectory.evaluate(sample/1000.);
      const auto mass=model.mass(point.q,state.I_total,state.m_total,state.F_x_Ctotal);
      const auto coriolis=model.coriolis(point.q,point.dq,state.I_total,state.m_total,state.F_x_Ctotal,state.O_ddP_O);
      const auto gravity=model.gravity(point.q,state.m_total,state.F_x_Ctotal,state.O_ddP_O);
      finite(mass);finite(coriolis);finite(gravity);
      m80::Joints dynamic{};
      for(std::size_t j=0;j<7;++j) {
        dynamic[j]=coriolis[j];
        for(std::size_t k=0;k<7;++k)dynamic[j]+=mass[j+7*k]*point.ddq[k];
        max_total[j]=std::max(max_total[j],std::abs(dynamic[j]+gravity[j]));
        max_dynamic[j]=std::max(max_dynamic[j],std::abs(dynamic[j]));
        if(!first)max_rate[j]=std::max(max_rate[j],std::abs(dynamic[j]-previous[j])/.001);
        if(max_total[j]>(j<4?87.:12.) || max_rate[j]>1000.)passed=false;
      }
      finite(dynamic);previous=dynamic;first=false;
    }
    finite(max_total);finite(max_dynamic);finite(max_rate);
    std::cout<<std::setprecision(17)<<"{\"read_only\":true,\"hardware_ready\":false,\"motion_executed\":false,\"kinematic_envelope_fraction\":0.95,\"server_protocol_version\":"<<robot.serverVersion()
      <<",\"waypoints\":"<<points.size()<<",\"sample_rate_hz\":1000,\"samples\":"<<samples<<",\"duration_s\":"<<points.back().time_s
      <<",\"m_ee_kg\":"<<state.m_ee<<",\"m_load_kg\":"<<state.m_load<<",\"m_total_kg\":"<<state.m_total<<",\"I_total_column_major_kg_m2\":";array(state.I_total);
    std::cout<<",\"F_x_Ctotal_m\":";array(state.F_x_Ctotal);
    std::cout<<",\"gravity_m_s2\":";array(state.O_ddP_O);
    std::cout<<",\"max_total_model_torque_nm\":";array(max_total);
    std::cout<<",\"max_without_gravity_model_torque_nm\":";array(max_dynamic);
    std::cout<<",\"max_without_gravity_model_torque_rate_nm_s\":";array(max_rate);
    std::cout<<",\"sampled_model_torque_screen_pass\":"<<(passed?"true":"false")
      <<",\"scope\":\"SDK model with the robot-reported constant tool/load; sampled 1kHz inverse dynamics only. Excludes contacts, motor friction, controller tracking error and payload changes; not hardware execution certification.\"}\n";
    return passed?0:2;
  } catch(const std::exception& error) {
    std::cerr<<"Read-only SDK model audit failed: "<<error.what()<<'\n';return 1;
  }
}
