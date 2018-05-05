#pragma once

#include <chrono>
#include <iostream>
#include <tuple>
#include <Eigen/Dense>
#include <math.h>
#include <aikido/planner/kinodynamics/sampler/Sampler.hpp>

namespace ompl {
namespace base {
class GibbsSampler : public MyInformedSampler
{
public:
  ///
  /// Constructor for Gibbs Sampler
  ///
  /// @param si Space information pointer
  /// @param problem OMPL's problem definition
  /// @param levelSet Initial level set of the problem
  /// @param maxNumberCalls Max number of calls to the sampler
  /// @param sampler Sampler that inherits from Sampler.h
  /// @param sample_batch_size How many samples to get each time a new
  /// batch of samples is gotten
  ///
  GibbsSampler(
      const SpaceInformationPtr& si,
      const ProblemDefinitionPtr& problem,
      const double levelSet,
      const unsigned int maxNumberCalls,
      const int sampleBatchSize)
    : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
  {
    // Use the start positions as starting point of the algorithm
    // (This will always be inside the informed subspace)
    prev_sample_ = getStartState();
  }

  virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;

  virtual void updateLevelSet(const double level_set) override;

  Eigen::VectorXd prev_sample_;

private:
  ///
  /// Get one random uniform sample from the space
  ///
  /// @return Random uniform vector from the space
  ///
  virtual Eigen::VectorXd getRandomSample(
      double min, double max, const int dim);
};

class HitAndRunSampler : public GibbsSampler
{
public:
  HitAndRunSampler(
      const SpaceInformationPtr& si,
      const ProblemDefinitionPtr& problem,
      const double levelSet,
      const unsigned int maxNumberCalls,
      const int sampleBatchSize,
      const int numOfTries = 10000)
    : GibbsSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
    , numOfTries_(numOfTries)
  {
    diagonalLength_ = 0.0;
    for (uint i = 0; i < getSpaceDimension(); i++)
      diagonalLength_
          = diagonalLength_
            + (stateMax_[i] - stateMin_[i]) * (stateMax_[i] - stateMin_[i]);
    diagonalLength_ = std::sqrt(diagonalLength_);

    numOfPrevSamples_ = 6;
    prevSamples_.reserve(numOfPrevSamples_);
    headOfPrevSamples_ = -1;
  }

  virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;

  void pushPrevSamples(Eigen::VectorXd& sample)
  {
    if (prevSamples_.size() < numOfPrevSamples_)
    {
      prevSamples_.push_back(sample);
    }
    else
    {
      prevSamples_[headOfPrevSamples_] = sample;
    }
    headOfPrevSamples_ = (headOfPrevSamples_ + 1) % numOfPrevSamples_;
    selectPrevSample();
  }

  void selectPrevSample()
  {
    if (prevSamples_.size() < numOfPrevSamples_)
    {
      prev_sample_ = prevSamples_.back();
      return;
    }

    double p = uniRndGnr_.sample();
    int index = headOfPrevSamples_;
    if (p < 0.5)
    {
      if (p > 0.25)
        index = index - 1;
      else if (p > 0.125)
        index = index - 2;
      else if (p > 0.06125)
        index = index - 3;
      else if (p > 0.0306125)
        index = index - 4;
      else
        index = index - 5;

      index = (index + numOfPrevSamples_) % numOfPrevSamples_;
    }
    prev_sample_ = prevSamples_[index];
    //                prev_sample_ =
    //                samples.row(index).head(getSpaceDimension());
  }

protected:
  int numOfTries_;
  NormalRealRandomGenerator normRndGnr_;
  double diagonalLength_;
  uint numOfPrevSamples_;
  std::vector<Eigen::VectorXd> prevSamples_;
  int headOfPrevSamples_;
};
} // namespace base
} // namespace ompl

// #endif // OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
