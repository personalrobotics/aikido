#ifndef AIKIDO_UTIL_STRING_HPP_
#define AIKIDO_UTIL_STRING_HPP_

#include <vector>
#include <string>

namespace aikido {
namespace util {

/// Splits (tokenizes) a string into substrings that are divided by the given
/// delimiters.
///
/// Example:
/// \code
/// std::string text = "HERB, the robot.";
///
/// auto res1 = split(text); // {"HERB,", "the", "robot."}
/// auto res2 = split(text, ","); // {"HERB", " the robot."}
/// \endcode
///
/// \param[in] string Input string to be splitted.
/// \param[in] delimiters Delimiters.
/// \return The splitted substrings.
std::vector<std::string> split(
    const std::string& string, const std::string& delimiters = " ");

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_STRING_HPP_
