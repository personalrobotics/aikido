#ifndef AIKIDO_PLANNER_PLANTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_PLANTOCONFIGURATION_HPP_

#include "aikido/planner/Problem.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a single goal configuration.
class PlanToConfiguration : public Problem
{
public:
  class Result;

protected:
};

class PlanToConfiguration::Result : public Problem::Result
{
public:

protected:

};

} // namespace planner
} // namespace aikido
