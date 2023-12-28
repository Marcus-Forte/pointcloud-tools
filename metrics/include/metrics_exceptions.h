#pragma once

#include <exception>
#include <string>

namespace duna {
namespace metrics {
namespace exceptions {
class exception : public std::exception {
 public:
  exception() : msg_("exception::duna::metrics") {}
  virtual ~exception() = default;

  virtual const char* what() const throw() override final { return msg_.c_str(); }

 protected:
  std::string msg_;
};

class not_enough_input_points : public exception {
 public:
  not_enough_input_points() { msg_ += "::not_enough_input_points\n"; }
  virtual ~not_enough_input_points() = default;
};

class unable_to_process_mesh : public exception {
 public:
  unable_to_process_mesh() { msg_ += "::unable_to_process_mesh\n"; }
  virtual ~unable_to_process_mesh() = default;
};
}  // namespace exceptions
}  // namespace metrics
}  // namespace duna