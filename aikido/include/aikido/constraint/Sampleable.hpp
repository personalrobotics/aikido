#ifndef AIKIDO_CONSTRAINT_SAMPLEABLE_H_
#define AIKIDO_CONSTRAINT_SAMPLEABLE_H_

#include <limits>
#include <memory>
#include <boost/optional.hpp>

#include "../util/RNG.hpp"

namespace aikido {
namespace constraint {

template <class T> class SampleGenerator;
template <class T> class SampleableConstraint;

template <class T>
class SampleableConstraint {
public:
  virtual std::unique_ptr<SampleGenerator<T>> createSampleGenerator() const = 0;
};

template <class T>
class SampleGenerator {
public:
  virtual boost::optional<T> sample() = 0;
  virtual int getNumSamples() const = 0;
  virtual bool canSample() const = 0;
  static constexpr int NO_LIMIT = std::numeric_limits<int>::max();
};


} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_SAMPLEABLE_H_
