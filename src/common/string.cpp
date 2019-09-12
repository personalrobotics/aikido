#include "aikido/common/string.hpp"

namespace aikido {
namespace common {

//==============================================================================
std::vector<std::string> split(
    const std::string& string, const std::string& delimiters)
{
  std::vector<std::string> tokens;

  // Skip delimiters at beginning.
  std::string::size_type lastPos = string.find_first_not_of(delimiters, 0);

  // Find first "non-delimiter".
  std::string::size_type pos = string.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    // Found a token, add it to the vector.
    tokens.push_back(string.substr(lastPos, pos - lastPos));

    // Skip delimiters.  Note the "not_of"
    lastPos = string.find_first_not_of(delimiters, pos);

    // Find next "non-delimiter"
    pos = string.find_first_of(delimiters, lastPos);
  }

  return tokens;
}

} // namespace common
} // namespace aikido
