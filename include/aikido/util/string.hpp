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
/// auto res3 = split(text, "to"); // {"HERB, ", "he r", "b", "."}
/// \endcode
///
/// \param[in] string Input string to be splitted.
/// \param[in] delimiters Delimiters where any character in this string is
/// considered a delimiter, i.e. the string is interpreted as a set of delimiter
/// characters, not as a single multi-character delimiter.
/// \return The splitted substrings. Space and tap (\t) are the default.
std::vector<std::string> split(
    const std::string& string, const std::string& delimiters = " \t");

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_STRING_HPP_
