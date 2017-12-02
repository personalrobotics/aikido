#ifndef AIKIDO_COMMON_ALGORITHM_HPP_
#define AIKIDO_COMMON_ALGORITHM_HPP_

#include <vector>

namespace aikido {
namespace common {

/// If v compares less than lo, returns lo; otherwise if hi compares less than
/// v, returns hi; otherwise returns v. Uses operator < to compare the values.
///
/// The behavior is undefined if the value of lo is greater than hi.
///
/// \param[in] v The value to clamp.
/// \param[in] lo The lower bound to clamp v to.
/// \param[in] hi The upper bound to clamp v to.
/// \param[in] comp Comparison function object (i.e., an object that satisfies
/// the requirements of Compare) which returns ​true if the first argument is
/// less than the second. The signature of the comparison function should be
/// equivalent to \code bool cmp(const Type1& a, const Type2& b) \endcode. The
/// signature does not need to have const &, but the function object must not
/// modify the objects passed to it. The types Type1 and Type2 must be such that
/// an object of type T can be implicitly converted to both of them.
/// \return Reference to lo if v is less than lo, reference to hi if hi is less
/// than v, otherwise reference to v.
template <class T, class Compare>
const T& clamp(const T& v, const T& lo, const T& hi, Compare comp);

/// Same as above, but uses std::less<T> to compare the values.
template <class T>
const T& clamp(const T& v, const T& lo, const T& hi);

// TODO(JS): docstring
template <typename T>
std::vector<T> linspace(T start, T end, std::size_t n, bool endpoint = true);

// TODO(JS): docstring
template<typename T>
std::vector<T> arange(T start, T stop, T step = static_cast<T>(1u));

} // namespace common
} // namespace aikido

#include "aikido/common/detail/algorithm-impl.hpp"

#endif // AIKIDO_COMMON_ALGORITHM_HPP_
