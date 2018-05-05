#ifndef MY_BIT_STAR_H_
#define MY_BIT_STAR_H_

// OMPL
#include <fstream>
#include <ios>
#include <iostream>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"

namespace ompl {
namespace geometric {
class MyBITstar : public ompl::geometric::BITstar
{
public:
  typedef enum { LOAD_SAMPLES, SAVE_SAMPLES, RANDOM_SAMPLES } PlannerMode;
  MyBITstar(const ompl::base::SpaceInformationPtr& si);

  virtual ompl::base::PlannerStatus solve(
      const ompl::base::PlannerTerminationCondition& ptc) override;

  void initLogFile(std::string scenarioName, std::string samplerName, int id);

  ompl::base::PlannerStatus solve(double solveTime);

  ompl::base::PlannerStatus solveAfterLoadingSamples(
      std::string filename, double solveTime);

  ompl::base::PlannerStatus solveAndSaveSamples(
      std::string filename, double solveTime);

  std::string fromState(ompl::base::State* fromState);
  bool toState(std::string stateString, ompl::base::State* toState);

  uint64_t getSamplesGeneratedNum()
  {
    return samplesGeneratedNum_;
  }

private:
  PlannerMode mode_;
  std::ofstream out_;
  std::ofstream sampleSaveStream_;
  std::ifstream sampleLoadStream_;

  uint64_t samplesGeneratedNum_;
};

using MyBITstarPtr = std::shared_ptr<MyBITstar>;
}
}

#endif // MY_BIT_STAR_H_
