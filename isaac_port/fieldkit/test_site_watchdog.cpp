#include "site_watchdog.hpp"
#include <stdexcept>
int main() {
  using namespace std::chrono;
  const auto now=m80::SiteHeartbeat::Clock::now();m80::SiteHeartbeat h(now);
  if(h.ready() || h.aborted())throw std::runtime_error("Unexpected initial watchdog state");
  h.line("READY",now);h.update(now+milliseconds(249));
  if(!h.ready())throw std::runtime_error("Fresh heartbeat lost");
  h.update(now+milliseconds(251));
  if(h.ready() || !h.aborted())throw std::runtime_error("Stale heartbeat accepted");
  h.line("READY",now+milliseconds(252));
  if(h.ready())throw std::runtime_error("Abort latch cleared by heartbeat");
  m80::SiteHeartbeat stop(now);stop.line("STOP",now);
  if(!stop.aborted())throw std::runtime_error("Stop did not abort");
  m80::SiteHeartbeat eof(now);eof.disconnect();
  if(!eof.aborted())throw std::runtime_error("EOF did not abort");
  m80::SiteHeartbeat bad(now);bad.line("READY extra",now);
  if(!bad.aborted())throw std::runtime_error("Unknown command did not abort");
}
