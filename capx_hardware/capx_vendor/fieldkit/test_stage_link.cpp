#include "libfranka_stage.hpp"

// Force the stage object and its SDK references into the executable. Merely
// creating a static archive does not prove that those references can link.
int main() {
  auto volatile stage = &m80::run_joint_stage;
  return stage == nullptr;
}
