#pragma once
#include <franka/robot_state.h>
#include <stdexcept>

namespace m80 {
inline const char* robot_mode_name(franka::RobotMode mode) {
  switch(mode) {
    case franka::RobotMode::kOther:return "other";
    case franka::RobotMode::kIdle:return "idle";
    case franka::RobotMode::kMove:return "move";
    case franka::RobotMode::kGuiding:return "guiding";
    case franka::RobotMode::kReflex:return "reflex";
    case franka::RobotMode::kUserStopped:return "user_stopped";
    case franka::RobotMode::kAutomaticErrorRecovery:return "automatic_error_recovery";
  }
  return "unknown";
}
inline void require_idle_mode(franka::RobotMode mode) {
  if(mode!=franka::RobotMode::kIdle)
    throw std::runtime_error("FR3 must be idle before starting a calibrated stage");
}
} // namespace m80
