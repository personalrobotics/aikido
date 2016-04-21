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
template <template <class> class Factory, class BaseParameter, class TypeList>
struct ForOneOf {};

template <template <class> class Factory, class BaseParameter>
struct ForOneOf<Factory, BaseParameter, util::type_list<>>
{
  template <class... Parameters>
  static std::nullptr_t create(std::shared_ptr<BaseParameter> /* unused */,
                               Parameters&&... /* unused */)
  {
    return nullptr;
  }
};

template <template <class> class Factory, class BaseParameter,
          class Arg, class... Args>
struct ForOneOf<Factory, BaseParameter, util::type_list<Arg, Args...>>
{
  template <class... Parameters>
  static auto create(std::shared_ptr<BaseParameter> _delegate,
                     Parameters&&... _params)
    -> decltype(Factory<Arg>::create(
         std::dynamic_pointer_cast<Arg>(_delegate),
         std::forward<Parameters>(_params)...))
  {
    if (auto arg = std::dynamic_pointer_cast<Arg>(_delegate))
      return Factory<Arg>::create(
        std::move(arg), std::forward<Parameters>(_params)...);
    else
      return ForOneOf<
        Factory, BaseParameter, util::type_list<Args...>>::create(
          std::move(_delegate), std::forward<Parameters>(_params)...);
  }
};

} // namespace util
} // namespace aikido


#endif // ifndef AIKIDO_UTIL_METAPROGRAMMING_H_
