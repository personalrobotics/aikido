#ifndef AIKIDO_UTIL_METAPROGRAMMING_H_
#define AIKIDO_UTIL_METAPROGRAMMING_H_
#include <memory>

namespace aikido {
namespace util {

//=============================================================================
/// Wrapper for a variadic template parameter pack of types.
template <class... Types>
class type_list {};

//=============================================================================
template <class Pointee>
struct ForOneOf_shared_ptr
{
  using type = std::shared_ptr<Pointee>;

  template <class Derived>
  static std::shared_ptr<Derived> cast(std::shared_ptr<Pointee> _pointer)
  {
    return std::dynamic_pointer_cast<Derived>(std::move(_pointer));
  }
};

template <class Pointee>
struct ForOneOf_raw_ptr
{
  using type = Pointee*;

  template <class Derived>
  static Derived* cast(Pointee* _pointer)
  {
    return dynamic_cast<Derived*>(_pointer);
  }
};

template <
  template <class> class Factory,
  template <class> class Pointer,
  class BaseParameter,
  class TypeList>
struct ForOneOf {};

template <
  template <class> class Factory,
  template <class> class Pointer,
  class BaseParameter>
struct ForOneOf<Factory, Pointer, BaseParameter, util::type_list<>>
{
  template <class... Parameters>
  static std::nullptr_t create(
    typename Pointer<BaseParameter>::type /* unused */,
    Parameters&&... /* unused */)
  {
    return nullptr;
  }
};

template <
  template <class> class Factory,
  template <class> class Pointer,
  class BaseParameter,
  class Arg,
  class... Args>
struct ForOneOf<Factory, Pointer, BaseParameter, util::type_list<Arg, Args...>>
{
  template <class... Parameters>
  static auto create(typename Pointer<BaseParameter>::type _base,
                     Parameters&&... _params)
    -> decltype(Factory<Arg>::create(
         Pointer<BaseParameter>::template cast<Arg>(_base),
         std::forward<Parameters>(_params)...))
  {
    if (auto derived = Pointer<BaseParameter>::template cast<Arg>(_base))
      return Factory<Arg>::create(
        std::move(derived), std::forward<Parameters>(_params)...);
    else
      return ForOneOf<
        Factory, Pointer, BaseParameter, util::type_list<Args...>>::create(
          std::move(_base), std::forward<Parameters>(_params)...);
  }
};

} // namespace util
} // namespace aikido


#endif // ifndef AIKIDO_UTIL_METAPROGRAMMING_H_
