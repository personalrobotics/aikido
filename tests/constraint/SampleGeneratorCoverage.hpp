#ifndef AIKIDO_TESTS_CONSTRAINT_SAMPLEGENERATORCOVERAGE_H_
#define AIKIDO_TESTS_CONSTRAINT_SAMPLEGENERATORCOVERAGE_H_
#include <vector>
#include <gtest/gtest.h>
#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/constraint/Sampleable.hpp>

template <class Iterator>
testing::AssertionResult SampleGeneratorCoverage(
  aikido::constraint::SampleGenerator& _generator,
  const aikido::distance::DistanceMetric& _metric,
  const Iterator _beginTargets, const Iterator _endTargets,
  double _distanceThreshold, size_t _numSamples)
{
  std::vector<int> counts(std::distance(_beginTargets, _endTargets), 0);

  const auto stateSpace = _generator.getStateSpace();
  auto state = stateSpace->createState();

  for (size_t isample = 0; isample < _numSamples; ++isample)
  {
    if (!_generator.canSample())
      return ::testing::AssertionFailure() << "Unable to sample.";

    if (!_generator.sample(state))
      return ::testing::AssertionFailure() << "Failed sampling.";

    size_t itarget = 0;
    for (Iterator it = _beginTargets; it != _endTargets; ++it, ++itarget)
    {
      if (_metric.distance(state, *it) < _distanceThreshold)
        counts[itarget]++;
    }
  }

  for (auto count : counts)
  {
    if (count == 0)
      return ::testing::AssertionFailure() << "Missed one or more targets.";
  }
  
  return testing::AssertionSuccess();
}

#endif
