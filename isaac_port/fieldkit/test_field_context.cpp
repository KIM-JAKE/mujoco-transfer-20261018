#include "libfranka_stage.hpp"
#include <limits>
#include <stdexcept>

namespace {
m80::FieldContext valid_context() {
  m80::FieldContext site;
  site.field_planned=site.tool_verified=site.io_verified=site.torque_budget_verified=true;
  site.calibration_record="unit-test-only";
  site.commissioning_record="unit-test-only";
  site.expected_tool_mass_kg=.9;
  site.expected_load_mass_kg=0.;
  return site;
}
void rejected_before_motion(const m80::FieldContext& site) {
  m80::AtomicGates gates;
  bool caught=false;
  try {
    m80::with_shared_abort(gates,[&] { m80::validate_field_context(site); });
  } catch(const std::exception&) { caught=true; }
  if(!caught || !gates.abort_requested.load())
    throw std::runtime_error("Invalid field record did not propagate a shared abort");
}
}

int main() {
  rejected_before_motion(m80::FieldContext{});
  for(double value : {std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity(),-1.}) {
    auto site=valid_context();site.expected_tool_mass_kg=value;rejected_before_motion(site);
    site=valid_context();site.expected_load_mass_kg=value;rejected_before_motion(site);
  }
  for(double value : {std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity(),0.,.951}) {
    auto site=valid_context();site.command_fraction=value;rejected_before_motion(site);
  }
  auto site=valid_context();m80::AtomicGates gates;
  const auto result=m80::with_shared_abort(gates,[&] {
    m80::validate_field_context(site);return 42;
  });
  if(result!=42 || gates.abort_requested.load())
    throw std::runtime_error("Successful preflight was changed by the abort wrapper");
  bool caught=false;
  try {
    m80::with_shared_abort(gates,[] { throw std::runtime_error("Simulated pre-control read failure"); });
  } catch(const std::exception&) { caught=true; }
  if(!caught || !gates.abort_requested.load())
    throw std::runtime_error("Pre-control failure did not reach the other arm's gate");
  m80::with_shared_abort(gates,[] {});
  if(!gates.abort_requested.load())
    throw std::runtime_error("Shared stop was cleared without explicit recovery");
}
