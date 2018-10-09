#ifndef AIKIDO_TESTS_CONSTRAINT_SAMPLEGENERATORCOVERAGE_H_
#define AIKIDO_TESTS_CONSTRAINT_SAMPLEGENERATORCOVERAGE_H_
#include <vector>
#include <gtest/gtest.h>
#include <aikido/constraint/Sampleable.hpp>
#include <aikido/distance/DistanceMetric.hpp>

template <class Iterator>
testing::AssertionResult SampleGeneratorCoverage(
    aikido::constraint::SampleGenerator& _generator,
    const aikido::distance::DistanceMetric& _metric,
    const Iterator _beginTargets,
    const Iterator _endTargets,
    double _distanceThreshold,
    std::size_t _numSamples)
{
  std::vector<int> counts(std::distance(_beginTargets, _endTargets), 0);

  std::cout << counts.size() << " " << _numSamples << std::endl;

  const auto stateSpace = _generator.getStateSpace();
  auto state = stateSpace->createState();

  for (std::size_t isample = 0; isample < _numSamples; ++isample)
  {
    if (!_generator.canSample())
      return ::testing::AssertionFailure() << "Unable to sample.";

    if (!_generator.sample(state))
      return ::testing::AssertionFailure() << "Failed sampling.";

    Eigen::VectorXd tangent;
    stateSpace->logMap(state, tangent);
    // std::cout << tangent[0] << " " << tangent[1] << " " << tangent[2] << std::endl;

    std::size_t itarget = 0;
    for (Iterator it = _beginTargets; it != _endTargets; ++it)
    {
      stateSpace->logMap(*it, tangent);
      std::cout << tangent[0] << " " << tangent[1] << " " << tangent[2] << std::endl;

      if (_metric.distance(state, *it) < _distanceThreshold)
        counts[itarget]++;

      if (itarget == 0)
      {
        std::cout << _metric.distance(state, *it) << std::endl;
      }
      itarget++;
    }
  }

  // for (auto count : counts)
  for (std::size_t i = 0; i < counts.size(); ++i)
  {
    if (counts[i] == 0)
    {
      std::cout << i << std::endl;
      return ::testing::AssertionFailure() << "Missed one or more targets.";
    }
  }

  return testing::AssertionSuccess();
}

#endif
