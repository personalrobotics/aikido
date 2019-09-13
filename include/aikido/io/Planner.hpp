#ifndef AIKIDO_IO_PLANNER_HPP_
#define AIKIDO_IO_PLANNER_HPP_

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "aikido/statespace/StateSpace.hpp"
#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/SequenceMetaPlanner.hpp"

namespace aikido {
namespace io {

/// Format for the planner configuration file.
/// TODO (avk): Converge on the configuration file format.
/// planners:
///   RRTConnect:
///     name: RRTConnect
///     range: 0.1
///   BITstarR:
///     name: BITstar
///     connection: radius
///   BITstarK:
///     name: BITstar
///     connection: k-nearest


/// Reads a planner configuration file and sets up a SequenceMetaPlanner
/// with requested planners and associated planner parameters.
/// \param[in] space Statespace the planner is operating in.
/// \param[in] configurationFile Path to the planner configuration file.
aikido::planner::PlannerPtr constructPlanners(aikido::statespace::ConstStateSpacePtr space,
                                              const std::string& configurationFile);

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_PLANNER_HPP_
