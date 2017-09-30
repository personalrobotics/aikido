#ifndef AIKIDO_COMMON_METAPROGRAMMING_HPP_
#define AIKIDO_COMMON_METAPROGRAMMING_HPP_

#include <memory>

namespace aikido {
namespace common {

/// Wrapper for a variadic template parameter pack of types.
///
/// \tparam Types... list of types
template <class... Types>
class type_list
{
};

/// Call a template factory function based on runtime type of the first
/// argument to a function. This class has a \c create function that takes
/// a pointer to \c BaseParameter as its first parameter, optionally followed
/// by arbitrary other parameters. If the runtime type of that argument is in
/// \c TypeList, then it is cast to the \c Derived type and forwarded to the
/// the template class \c Factory<Derived>::create function. Other parameters,
/// and the return value of \c create, are perfectly forwarded. If the first
/// argument is none of those types, then \c create returns \c nullptr.
///
/// RTTI is implemented by attempting to \c dynamic_cast the first parameter of
/// \c create to each derived type in \c TypeList, in the order that they are
/// specified in. This is a potentially expensive operation.
///
/// The \c Pointer<Derived> parameter is a template class that defines: (1) a
/// pointer type \c Pointer<Derived>::type and (2) a corresponding
/// \c dynamic_cast operator \c Pointer<T>::cast that attempts to cast a
/// \c Pointer<BaseParameter>::type to a \c Pointer<Derived>>:type. The
/// provided \c DynamicCastFactory_shared_ptr and
/// \c DynamicCastFactory_raw_pointer classes implement this functionality for
/// \c std::shared_ptr and raw pointers, respectively.
///
/// \tparam Factory factory template class with a static \c create function
/// \tparam Pointer pointer type information
/// \tparam BaseParameter base class
/// \tparam TypeList \c type_list of subclasses of \c BaseParameter
template <template <class> class Factory,
          template <class> class Pointer,
          class BaseParameter,
          class TypeList>
struct DynamicCastFactory
{
};

/// Helper template class necessary to use \c std::shared_ptr as the pointer
/// type in \c DynamicCastFactory.
///
/// \tparam pointee type
template <class Pointee>
struct DynamicCastFactory_shared_ptr;

/// Helper template class necessary to use raw pointers as the pointer type
/// in \c DynamicCastFactory.
///
/// \tparam pointee type
template <class Pointee>
struct DynamicCastFactory_raw_ptr;

} // namespace common
} // namespace aikido

#include "detail/metaprogramming-impl.hpp"

#endif // AIKIDO_COMMON_METAPROGRAMMING_HPP_
