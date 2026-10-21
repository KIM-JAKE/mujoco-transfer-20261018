#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace m80 {
// Raw callback diagnostics, including a callback that subsequently fails a
// guard. Units: seconds, rad, rad/s, rad/s^2, Nm; wrench is N then Nm.
// O_T_EE uses the SDK column-major convention. q_d is the controller's
// reported desired state, not the unfiltered application trajectory.
struct TelemetrySample {
  std::uint64_t sample_index=0;
  double elapsed_s=0.,robot_time_s=0.,period_s=0.;
  std::array<double,7> q{},dq{},q_d{},dq_d{},ddq_d{},tau_J{},tau_J_d{},tau_ext_hat_filtered{};
  std::array<double,6> O_F_ext_hat_K{};
  std::array<double,16> O_T_EE{};
  int robot_mode=0;
  bool current_errors=false,io_ready=false,abort_requested=false;
};

// Exactly one producer (one arm's callback) and one consumer. Construct the
// queue outside the callback and keep it alive until both have finished.
// Overflow drops the new sample; the producer never waits for a consumer.
template<std::size_t Capacity>
class TelemetryQueue {
  static_assert(Capacity>=2,"Telemetry queue needs at least two slots");
  static_assert(std::atomic<std::size_t>::is_always_lock_free,"Lock-free queue indices required");
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free,"Lock-free drop counter required");
 public:
  TelemetryQueue()=default;
  TelemetryQueue(const TelemetryQueue&)=delete;
  TelemetryQueue& operator=(const TelemetryQueue&)=delete;
  bool try_push(const TelemetrySample& sample) noexcept {
    const auto head=head_.load(std::memory_order_relaxed);
    const auto next=(head+1)%Capacity;
    if(next==tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1,std::memory_order_relaxed);
      return false;
    }
    samples_[head]=sample;
    head_.store(next,std::memory_order_release);
    return true;
  }
  bool try_pop(TelemetrySample& sample) noexcept {
    const auto tail=tail_.load(std::memory_order_relaxed);
    if(tail==head_.load(std::memory_order_acquire))return false;
    sample=samples_[tail];
    tail_.store((tail+1)%Capacity,std::memory_order_release);
    return true;
  }
  std::uint64_t dropped_samples() const noexcept {
    return dropped_.load(std::memory_order_relaxed);
  }
 private:
  std::array<TelemetrySample,Capacity> samples_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  std::atomic<std::uint64_t> dropped_{0};
};
using StageTelemetry=TelemetryQueue<512>;
} // namespace m80
