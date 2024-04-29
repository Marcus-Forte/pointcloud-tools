#include <stdexcept>

namespace duna
{
  class unimplemented_exception : std::runtime_error
  {
    public:
    unimplemented_exception(char const* const message):std::runtime_error(message){}
    const char* what() const noexcept override { return std::runtime_error::what();}
  };

  class invalid_argument_exception : std::runtime_error
  {
    public:
    invalid_argument_exception(char const* const message):std::runtime_error(message){}
    const char* what() const noexcept override { return std::runtime_error::what();}
  };

  class aborted_exception : std::runtime_error
  {
    public:
    aborted_exception(char const* const message):std::runtime_error(message){}
    const char* what() const noexcept override { return std::runtime_error::what();}
  };

} // namespace duna

