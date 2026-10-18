#include "libfranka_stage.hpp"
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/control_types.h>
#include <stdexcept>

namespace m80 {
void validate_field_context(const FieldContext& site) {
  if(!site.field_planned || !site.tool_verified || !site.io_verified || !site.torque_budget_verified ||
     site.calibration_record.empty() || site.commissioning_record.empty())
    throw std::runtime_error("Field commissioning evidence is incomplete; simulation results cannot authorize motion");
  if(!std::isfinite(site.expected_tool_mass_kg) || !std::isfinite(site.expected_load_mass_kg) ||
     site.expected_tool_mass_kg<0 || site.expected_load_mass_kg<0)
    throw std::runtime_error("Verified payload masses are required");
  if(!std::isfinite(site.command_fraction) || site.command_fraction<=0 || site.command_fraction>.95)
    throw std::runtime_error("Command fraction must be finite and in (0,0.95]");
}

ServoReport run_joint_stage(franka::Robot& robot,const Trajectory& path,
                           const FieldContext& site,AtomicGates& gates) {
  return with_shared_abort(gates,[&]() -> ServoReport {
    validate_field_context(site);
    path.validate(site.command_fraction);
    if(robot.serverVersion()!=10)
      throw std::runtime_error("This motion envelope requires FR3 protocol 10 / System Image >=5.9; replan for other firmware");
    const auto initial=robot.readOnce();
    if(initial.current_errors || !gates.io_ready.load() || gates.abort_requested.load())
      throw std::runtime_error("Robot or field I/O is not ready");
    if(!std::isfinite(initial.m_ee) || !std::isfinite(initial.m_load) ||
       std::abs(initial.m_ee-site.expected_tool_mass_kg)>.001 ||
       std::abs(initial.m_load-site.expected_load_mass_kg)>.001)
      throw std::runtime_error("Configured robot payload differs from field record");
    const auto start=path.evaluate(0.).q;
    for(std::size_t j=0;j<7;++j)
      if(!std::isfinite(initial.q_d[j]) || !std::isfinite(initial.q[j]) ||
         !std::isfinite(initial.dq[j]) || !std::isfinite(initial.tau_J_d[j]) ||
         std::abs(initial.q_d[j]-start[j])>1e-6 || std::abs(initial.q[j]-start[j])>.005 || std::abs(initial.dq[j])>.01)
        throw std::runtime_error("Robot is not stationary at the field-planned stage start; no automatic jump or homing");
    ServoReport report;
    double time=0.;Joints previous_torque=initial.tau_J_d;bool first=true;
    robot.control([&](const franka::RobotState& state,franka::Duration period)->franka::JointPositions {
      const double dt=period.toSec();
      if(gates.abort_requested.load(std::memory_order_relaxed) || !gates.io_ready.load(std::memory_order_relaxed))
        throw std::runtime_error("Field interlock lost");
      if(!first && (dt<.0005 || dt>.0015))throw std::runtime_error("FCI update did not meet the 1kHz stage timing contract");
      for(std::size_t j=0;j<7;++j)
        if(!std::isfinite(state.q[j]) || !std::isfinite(state.q_d[j]) ||
           !std::isfinite(state.dq[j]) || !std::isfinite(state.tau_J_d[j]))
          throw std::runtime_error("Non-finite robot state in stage callback");
      if(first) {
        for(std::size_t j=0;j<7;++j)
          if(std::abs(state.q_d[j]-start[j])>1e-6)throw std::runtime_error("Stage start changed before control became active");
        previous_torque=state.tau_J_d;
      } else {
        time+=dt;
        for(std::size_t j=0;j<7;++j) {
          const double rate=std::abs(state.tau_J_d[j]-previous_torque[j])/dt;
          report.maximum_torque_rate_nm_s=std::max(report.maximum_torque_rate_nm_s,rate);
          if(rate>1000.)throw std::runtime_error("Measured desired torque rate exceeds FR3 limit");
        }
      }
      first=false;previous_torque=state.tau_J_d;
      const auto command=path.evaluate(std::min(time,path.duration()));
      ++report.samples;report.maximum_period_s=std::max(report.maximum_period_s,dt);
      for(std::size_t j=0;j<7;++j) {
        const double error=std::abs(state.q[j]-command.q[j]);
        report.maximum_tracking_error_rad=std::max(report.maximum_tracking_error_rad,error);
        if(error>.02)throw std::runtime_error("Measured joint tracking error exceeds 0.02rad");
      }
      franka::JointPositions target(command.q);
      if(time>=path.duration())return franka::MotionFinished(target);
      return target;
    },franka::ControllerMode::kJointImpedance,true,100.);
    report.completed=true;
    return report;
  });
}
} // namespace m80
