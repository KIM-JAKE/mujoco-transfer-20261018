#pragma once
#include <chrono>
#include <string>
namespace m80 {
class SiteHeartbeat {
 public:
  using Clock=std::chrono::steady_clock;
  explicit SiteHeartbeat(Clock::time_point now):last_(now) {}
  void line(const std::string& value,Clock::time_point now) {
    if(value!="READY" || aborted_) {disconnect();return;}
    last_=now;ready_=true;
  }
  void update(Clock::time_point now) {
    if(now-last_>std::chrono::milliseconds(250))disconnect();
  }
  void disconnect() {ready_=false;aborted_=true;}
  bool ready()const {return ready_;}
  bool aborted()const {return aborted_;}
 private:
  Clock::time_point last_;
  bool ready_=false,aborted_=false;
};
}
