#ifndef AIKIDO_CONSTRAINT_SAMPLEABLE_H_
#define AIKIDO_CONSTRAINT_SAMPLEABLE_H_

#include <limits>
#include <memory>
#include <boost/optional.hpp>

#include "../util/RNG.hpp"

namespace aikido {
namespace constraint {

template <class T> class SampleGenerator;


/// Constraint that may be sampled from. To draw a sample from this constraint,
/// use createSampleGenerator() to create a SampleGenerator<T> and call its
/// sample() method. SampleGenerator<T> is entirely deterministic and all
/// generators constructed by this object will return the same sequence of
/// samples. For that reason, you should be careful to use the same
/// SampleGenerator<T> when obtaining a sequence of samples to avoid
/// re-sampling the beginning of the same deterministic sequence repeatedly.
template <class T>
class SampleableConstraint {
public:
  using SampleGeneratorPtr = std::unique_ptr<SampleGenerator<T>>;

  /// Creates a SampleGenerator for sampling from this constraint.
  virtual SampleGeneratorPtr createSampleGenerator() const = 0;
};


/// Generator for drawing samples from a SampleableConstraint<T>. This object
/// may represent both finite and inifinite sets of samples, as indicated by
/// the return value of getNumSamples(). Note that this value provides only an
/// upper bound on the number of samples available: sample() may transiently
/// fail, i.e. return an empty optional<T>, at any point before then.
template <class T>
class SampleGenerator {
public:
  /// Returns one sample from this constraint or failure.
  virtual boost::optional<T> sample() = 0;

  /// Gets an upper bound on the number of samples remaining or NO_LIMIT.
  virtual int getNumSamples() const = 0;

  /// Returns whether getNumSamples() > 0.
  virtual bool canSample() const = 0;

  /// Value used to represent a potentially infinite number of samples.
  static constexpr int NO_LIMIT = std::numeric_limits<int>::max();
};


} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_SAMPLEABLE_H_
