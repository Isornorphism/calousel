#ifndef CALOUSEL__LOGGING_HPP_
#define CALOUSEL__LOGGING_HPP_

#include <functional>
#include <iostream>
#include <string>

namespace calousel {

struct Logger {
  std::function<void(const std::string&)> info;
  std::function<void(const std::string&)> warn;
  std::function<void(const std::string&)> error;
};

inline Logger stderr_logger() {
  Logger log;
  log.info = [](const std::string& s) { std::cerr << "[INFO] " << s << "\n"; };
  log.warn = [](const std::string& s) { std::cerr << "[WARN] " << s << "\n"; };
  log.error = [](const std::string& s) { std::cerr << "[ERROR] " << s << "\n"; };
  return log;
}

}  // namespace calousel

#endif
