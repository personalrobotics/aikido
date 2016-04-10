#ifndef AIKIDO_UTIL_RNG_H_
#define AIKIDO_UTIL_RNG_H_

#include <cassert>
#include <cmath>
#include <memory>
#include <random>
#include <cstdint>
#include <Eigen/Geometry>

namespace aikido {
namespace util {

/// Implementation of the C++11 "random engine" concept that uses virtual
/// function calls to erase the type of the underlying random engine.
class RNG
{
public:
  using result_type = std::uint32_t;

  /// Number of bits the generated numbers should have. Derived types are
  /// responsible for providing numbers in this range, if necessary, by
  /// wrapping their engine in a std::independent_bits_engine.
  static constexpr std::size_t NUM_BITS = 32;

  virtual ~RNG() = default;

  /// Gets the smallest possible value in the output range, always zero.
  static constexpr result_type min()
  {
    return 0u;
  }

  /// Gets the largest possible value in the output range, 2^NUM_BITS - 1.
  static constexpr result_type max()
  {
    return (static_cast<std::uint64_t>(1) << NUM_BITS) - 1;
  }

  /// Advances the state of the engine and returns the generated value.
  virtual result_type operator()() = 0;

  /// Advances the adaptor's state by a specified amount.
  virtual void discard(unsigned long long _z) = 0;

  /// Create a copy of this RNG, including its internal state.
  virtual std::unique_ptr<RNG> clone() const = 0;

  /// Create a new RNG of this type with the specified seed.
  virtual std::unique_ptr<RNG> clone(result_type _seed) const = 0;
};


/// Concrete implementation of the RNG type erasure class.
template <class T>
class RNGWrapper : virtual public RNG
{
public:
  using RNG::result_type;
  using engine_type = std::independent_bits_engine<
    T, RNG::NUM_BITS, result_type>;

  /// Initialize a random engine with a default seed.
  RNGWrapper() = default;

  /// Initialize a random engine with the state from an existing random engine.
  explicit RNGWrapper(const T& _rng)
    : mRng(_rng)
  {
  }

  /// Constructs a random engine with the specified seed.
  explicit RNGWrapper(result_type _seed)
    : mRng(_seed)
  {
  }

  virtual ~RNGWrapper() = default;

  /// Gets the internal random engine.
  T& rng()
  {
    return mRng.base();
  }

  /// Gets the internal random engine.
  const T& rng() const
  {
    return mRng.base();
  }

  // Documentation inherited.
  result_type operator()() override
  {
    return mRng();
  }

  // Documentation inherited.
  void discard(unsigned long long _z) override
  {
    mRng.discard(_z);
  }
  
  // Documentation inherited.
  std::unique_ptr<RNG> clone() const override
  {
    return std::unique_ptr<RNGWrapper>(new RNGWrapper(mRng.base()));
  }

  std::unique_ptr<RNG> clone(result_type _seed) const override
  {
    return std::unique_ptr<RNGWrapper>(new RNGWrapper(_seed));
  }

private:
  engine_type mRng;
};

/// Sample a unit quaternion uniformly at random. This function requires that
/// the provided std::uniform_real_distribution has bounds of [ 0, 1 ].
template <
  class Engine, class Scalar, class Quaternion = Eigen::Quaternion<Scalar>>
Quaternion sampleQuaternion(
  Engine& _engine, std::uniform_real_distribution<Scalar>& _distribution)
{
  assert(_distribution.a() == 0.);
  assert(_distribution.b() == 1.);

  const double u1 = _distribution(_engine);
  const double u2 = _distribution(_engine);
  const double u3 = _distribution(_engine);

  return Quaternion(
    std::sqrt(1. - u1) * std::sin(2 * M_PI * u2),
    std::sqrt(1. - u1) * std::cos(2 * M_PI * u2),
    std::sqrt(u1) * std::sin(2 * M_PI * u3),
    std::sqrt(u1) * std::cos(2 * M_PI * u3)
  );
}

} // util
} // aikido

#endif // AIKIDO_UTIL_RNG_H_
