#include "telemetry.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

void require(bool value,const char* reason) {
  if(!value)throw std::runtime_error(reason);
}
int main() {
  // Overflow is bounded and counted; an unread record is never overwritten.
  m80::TelemetryQueue<3> tiny;
  m80::TelemetrySample input,output;
  input.sample_index=1;require(tiny.try_push(input),"first sample");
  input.sample_index=2;require(tiny.try_push(input),"second sample");
  input.sample_index=3;require(!tiny.try_push(input),"full queue must drop");
  require(tiny.dropped_samples()==1,"drop count");
  require(tiny.try_pop(output)&&output.sample_index==1,"preserve oldest");
  require(tiny.try_push(input),"reuse consumed slot");
  require(tiny.try_pop(output)&&output.sample_index==2,"second still intact");
  require(tiny.try_pop(output)&&output.sample_index==3,"wrapped slot intact");
  require(!tiny.try_pop(output),"empty queue");

  // Incident diagnostics retain raw nonfinite values and readiness flags.
  input.period_s=std::nan("");input.io_ready=false;input.abort_requested=true;
  require(tiny.try_push(input)&&tiny.try_pop(output),"raw incident capture");
  require(std::isnan(output.period_s)&&!output.io_ready&&output.abort_requested,"preserve incident values");

  // Exercise producer/consumer handoff through repeated wraps. Every field
  // carries the record index so a torn or prematurely overwritten copy fails.
  m80::TelemetryQueue<17> queue;
  std::atomic<bool> bad{false};
  constexpr std::uint64_t count=100000;
  std::thread producer([&] {
    for(std::uint64_t i=0;i<count;++i) {
      m80::TelemetrySample sample;sample.sample_index=i;
      sample.q.fill(double(i));sample.dq.fill(-double(i));
      sample.tau_J_d.fill(double(i)+.25);sample.O_T_EE.fill(double(i)+.5);
      sample.period_s=.001;sample.io_ready=true;
      while(!queue.try_push(sample))std::this_thread::yield();
    }
  });
  std::thread consumer([&] {
    for(std::uint64_t i=0;i<count;++i) {
      m80::TelemetrySample sample;
      while(!queue.try_pop(sample))std::this_thread::yield();
      bool valid=sample.sample_index==i&&sample.period_s==.001&&sample.io_ready;
      for(auto v:sample.q)valid=valid&&v==double(i);
      for(auto v:sample.dq)valid=valid&&v==-double(i);
      for(auto v:sample.tau_J_d)valid=valid&&v==double(i)+.25;
      for(auto v:sample.O_T_EE)valid=valid&&v==double(i)+.5;
      if(!valid)bad.store(true);
    }
  });
  producer.join();consumer.join();
  require(!bad.load(),"concurrent record integrity/order");
  require(!queue.try_pop(output),"all records consumed");
  std::cout<<"Telemetry overflow, incident and 100000-record concurrent tests passed\n";
}
