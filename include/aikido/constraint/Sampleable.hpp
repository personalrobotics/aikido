#ifndef AIKIDO_CONSTRAINT_SAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_SAMPLEABLE_HPP_

#include <limits>
#include <memory>
#include <boost/optional.hpp>

#include "aikido/common/pointers.hpp"
#include "../common/RNG.hpp"
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace constraint {

AIKIDO_DECLARE_POINTERS(Sampleable)

class SampleGenerator;

/// Constraint that may be sampled from. To draw a sample from this constraint,
/// use createSampleGenerator() to create a SampleGenerator and call its
/// sample() method. SampleGenerator is entirely deterministic and all
/// generators constructed by this object will return the same sequence of
/// samples. For that reason, you should be careful to use the same
/// SampleGenerator when obtaining a sequence of samples to avoid
/// re-sampling the beginning of the same deterministic sequence repeatedly.
class Sampleable
{
public:
  virtual ~Sampleable() = default;

  /// Gets the StateSpace that this constraint operates on.
  virtual statespace::ConstStateSpacePtr getStateSpace() const = 0;

  /// Creates a SampleGenerator for sampling from this constraint.
  virtual std::unique_ptr<SampleGenerator> createSampleGenerator() const = 0;
};

/// Generator for drawing samples from a Sampleable. This object
/// may represent both finite and inifinite sets of samples, as indicated by
/// the return value of getNumSamples(). Note that this value provides only an
/// upper bound on the number of samples available: sample() may transiently
/// fail, i.e. return false, at any point before then.
class SampleGenerator
{
public:
  virtual ~SampleGenerator() = default;

  /// Value used to represent a potentially infinite number of samples.
  static constexpr int NO_LIMIT = std::numeric_limits<int>::max();

  /// Gets the StateSpace that this SampleGenerator samples from.
  virtual statespace::ConstStateSpacePtr getStateSpace() const = 0;

  /// Returns one sample from this constraint; returns true if succeeded.
  virtual bool sample(statespace::StateSpace::State* _state) = 0;

  /// Gets an upper bound on the number of samples remaining or NO_LIMIT.
  virtual int getNumSamples() const = 0;

  /// Returns whether getNumSamples() > 0.
  virtual bool canSample() const = 0;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_SAMPLEABLE_HPP_
