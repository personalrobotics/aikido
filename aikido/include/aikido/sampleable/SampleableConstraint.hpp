#ifndef AIKIDO_SAMPLEABLE_H_
#define AIKIDO_SAMPLEABLE_H_

#include <limits>
#include <memory>
#include <boost/optional.hpp>

#include "../util/RNG.hpp"

namespace aikido {
namespace sampleable {

using aikido::util::RNG;
using namespace boost;

template <class T>
class SampleGenerator {
public:
  // returns T. If all samples are exhausted, return empty optional.
  virtual optional<T> sample() = 0;
  virtual int numSamples() = 0;
  virtual bool canSample() = 0;
  static const int NO_LIMIT = std::numeric_limits<int>::max();
};


template <class T>
class SampleableConstraint {
public:
  virtual std::unique_ptr<SampleGenerator<T>> sampler() const = 0;
  virtual bool isSatisfied(const T) const = 0;
};

} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_SAMPLEABLE_H_

