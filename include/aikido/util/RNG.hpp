#ifndef AIKIDO_UTIL_RNG_HPP_
#define AIKIDO_UTIL_RNG_HPP_

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <Eigen/Geometry>

namespace aikido {
namespace util {

/// Default number of seeds to by \c splitEngine to seed new engines.
constexpr int NUM_DEFAULT_SEEDS{100};

/// Implementation of the C++11 "random engine" concept that uses virtual
/// function calls to erase the type of the underlying random engine.
class RNG
{
public:
  using result_type = std::uint32_t;

  /// Number of bits the generated numbers should have. Derived types are
  /// responsible for providing numbers in this range, if necessary, by
  /// wrapping their engine in a std::independent_bits_engine.
  static constexpr std::size_t NUM_BITS{32};

  virtual ~RNG() = default;

  /// Gets the smallest possible value in the output range, always zero.
  ///
  /// \return smallest possible value in the output range
  static constexpr result_type min();

  /// Gets the largest possible value in the output range, 2^NUM_BITS - 1.
  ///
  /// \return largest possible value in the output range
  static constexpr result_type max();

  /// Advances the state of the engine and returns the generated value.
  ///
  /// \return random value
  virtual result_type operator()() = 0;

  /// Advances the adaptor's state by a specified amount.
  ///
  /// \param _z amount of state to discard
  virtual void discard(unsigned long long _z) = 0;

  /// Create a copy of this RNG, including its internal state.
  ///
  /// \return copy of this engine
  virtual std::unique_ptr<RNG> clone() const = 0;

  /// Create a new RNG of this type with the specified seed.
  ///
  /// \param _seed new sed
  /// \return new RNG constructed with the specified seed
  virtual std::unique_ptr<RNG> clone(result_type _seed) const = 0;
};

/// Concrete implementation of the RNG type erasure class.
///
/// \tparam type of random engine to wrap
template <class T>
class RNGWrapper : virtual public RNG
{
public:
  using RNG::result_type;
  using engine_type
      = std::independent_bits_engine<T, RNG::NUM_BITS, result_type>;

  /// Constructs a random engine with a default seed.
  RNGWrapper() = default;

  /// Constructs a random engine with the state from an existing random engine.
  ///
  /// \param _rng instance to use for initialization
  explicit RNGWrapper(const T& _rng);

  /// Constructs a random engine with the specified seed.
  ///
  /// \param _seed initial seed to use for the random engine
  explicit RNGWrapper(result_type _seed);

  virtual ~RNGWrapper() = default;

  /// Gets the internal random engine.
  ///
  /// \return internal random engine
  T& rng();

  /// Gets the internal random engine.
  ///
  /// \return internal random engine
  const T& rng() const;

  // Documentation inherited.
  result_type operator()() override;

  // Documentation inherited.
  void discard(unsigned long long _z) override;

  // Documentation inherited.
  std::unique_ptr<RNG> clone() const override;

  // Documentation inherited.
  std::unique_ptr<RNG> clone(result_type _seed) const override;

private:
  engine_type mRng;
};

/// Sample a unit quaternion uniformly at random. This function requires that
/// the provided std::uniform_real_distribution has bounds of [ 0, 1 ].
///
/// \tparam Engine type of random engine
/// \tparam Scalar type of floating point scalar number
/// \tparam Quaternion type of quaternion to create
/// \param _engine random engine
/// \param _distribution uniform distribution over the range [ 0, 1 ]
/// \return sampled unit quaternion
template <class Engine,
          class Scalar,
          class Quaternion = Eigen::Quaternion<Scalar>>
Quaternion sampleQuaternion(
    Engine& _engine, std::uniform_real_distribution<Scalar>& _distribution);

/// Deterministically create different \c _numOutputs random number generators
/// of the same type as the input \c _engine. This is implemented by using
/// \c _engine to generate \c _numSeeds seeds, then using \c std::seed_seq to
/// generate \c _numOutputs uncorrelated seeds to create the output engines.
///
/// \param _engine random engine
/// \param _numOutputs number of RNGs to create
/// \param _numSeeds number of seeds to use for initialization
/// \return new random number generators
std::vector<std::unique_ptr<util::RNG>> splitEngine(
    RNG& _engine, size_t _numOutputs = 1, size_t _numSeeds = NUM_DEFAULT_SEEDS);

} // namespace util
} // namespace aikido

#include "detail/RNG-impl.hpp"

#endif // AIKIDO_UTIL_RNG_HPP_
