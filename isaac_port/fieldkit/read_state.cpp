// Read-only FCI diagnostic. No control(), recovery or configuration writes.
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>

template <std::size_t N>
void array_json(const std::array<double, N>& a) {
  std::cout << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i) std::cout << ',';
    std::cout << a[i];
  }
  std::cout << ']';
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: m80_read_state ROBOT_IP\n";
    return 2;
  }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    const auto state = robot.readOnce();
    std::cout << std::setprecision(17)
              << "{\"read_only\":true,\"hardware_motion_tested\":false,\"server_protocol_version\":"
              << robot.serverVersion() << ",\"q\":";
    array_json(state.q);
    std::cout << ",\"q_d\":"; array_json(state.q_d);
    std::cout << ",\"dq\":"; array_json(state.dq);
    std::cout << ",\"dq_d\":"; array_json(state.dq_d);
    std::cout << ",\"ddq_d\":"; array_json(state.ddq_d);
    std::cout << ",\"tau_J\":"; array_json(state.tau_J);
    std::cout << ",\"tau_J_d_without_gravity\":"; array_json(state.tau_J_d);
    std::cout << ",\"O_T_EE_column_major\":"; array_json(state.O_T_EE);
    std::cout << ",\"F_T_EE_column_major\":"; array_json(state.F_T_EE);
    std::cout << ",\"O_F_ext_hat_K\":"; array_json(state.O_F_ext_hat_K);
    std::cout << ",\"I_total_column_major_kg_m2\":"; array_json(state.I_total);
    std::cout << ",\"F_x_Ctotal_m\":"; array_json(state.F_x_Ctotal);
    std::cout << ",\"base_gravity_m_s2\":"; array_json(state.O_ddP_O);
    std::cout << ",\"m_ee_kg\":" << state.m_ee
              << ",\"m_load_kg\":" << state.m_load
              << ",\"m_total_kg\":" << state.m_total
              << ",\"robot_mode\":" << static_cast<int>(state.robot_mode)
              << ",\"control_command_success_rate\":" << state.control_command_success_rate
              << "}\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Read-only FCI probe failed: " << e.what() << '\n';
    return 1;
  }
}
