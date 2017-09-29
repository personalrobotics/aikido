#include <cassert>

namespace aikido {
namespace util {

//==============================================================================
constexpr auto RNG::min() -> result_type
{
  return 0u;
}

//==============================================================================
constexpr auto RNG::max() -> result_type
{
  return (static_cast<std::uint64_t>(1) << NUM_BITS) - 1;
}

//==============================================================================
template <class T>
RNGWrapper<T>::RNGWrapper(const T& _rng) : mRng(_rng)
{
}

//==============================================================================
template <class T>
RNGWrapper<T>::RNGWrapper(result_type _seed) : mRng(_seed)
{
}

//==============================================================================
template <class T>
auto RNGWrapper<T>::rng() -> T&
{
  // This is potentially dangerous, but is necessary to implement the current
  // API. We should deprecate this function in the future.
  return const_cast<T&>(mRng.base());
}

//==============================================================================
template <class T>
auto RNGWrapper<T>::rng() const -> const T&
{
  return mRng.base();
}

//==============================================================================
template <class T>
auto RNGWrapper<T>::operator()() -> result_type
{
  return mRng();
}

//==============================================================================
template <class T>
void RNGWrapper<T>::discard(unsigned long long _z)
{
  mRng.discard(_z);
}

//==============================================================================
template <class T>
std::unique_ptr<RNG> RNGWrapper<T>::clone() const
{
  return std::unique_ptr<RNGWrapper>(new RNGWrapper(mRng.base()));
}

//==============================================================================
template <class T>
std::unique_ptr<RNG> RNGWrapper<T>::clone(result_type _seed) const
{
  return std::unique_ptr<RNGWrapper>(new RNGWrapper(_seed));
}

//==============================================================================
template <class Engine, class Scalar, class Quaternion>
Quaternion sampleQuaternion(
    Engine& _engine, std::uniform_real_distribution<Scalar>& _distribution)
{
  assert(_distribution.a() == 0.);
  assert(_distribution.b() == 1.);

  const double u1 = _distribution(_engine);
  const double u2 = _distribution(_engine);
  const double u3 = _distribution(_engine);

  return Quaternion(
      std::sqrt(1. - u1) * std::sin(2. * M_PI * u2),
      std::sqrt(1. - u1) * std::cos(2. * M_PI * u2),
      std::sqrt(u1) * std::sin(2. * M_PI * u3),
      std::sqrt(u1) * std::cos(2. * M_PI * u3));
}

} // namespace util
} // namespace aikido
