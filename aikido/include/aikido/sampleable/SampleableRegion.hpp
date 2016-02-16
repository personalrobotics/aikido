#ifndef AIKIDO_SAMPLEABLE_H_
#define AIKIDO_SAMPLEABLE_H_

#include "../util/RNG.hpp"
#include <limits>
namespace aikido {
namespace sampleable {


template <typename T> 
class SampleableRegion{
public:

	/// Decide whether a given T is part of the region.
	virtual bool isSatisfied(const T t) const;

	/// Sample T in the region.
	virtual const T sample(aikido::util::RNG& rng);

	/// Return true if maxSampleCount() > 0
	virtual bool canSample() const;

	/// Return the maximum number of samples that can be asked for before repeating.
	virtual int maxSampleCount() const;

	static const int INFTY = std::numeric_limits<int>::max();
};

} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_SAMPLEABLE_H_