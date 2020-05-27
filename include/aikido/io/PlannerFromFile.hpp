#ifndef AIKIDO_IO_PLANNERFROMFILE_HPP_
#define AIKIDO_IO_PLANNERFROMFILE_HPP_

#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace io {
/*
Read a planner configuration file to load planners and their parameters.
By default, all the planners are loaded as base aikido::planner::Planner type
and stored into a vector.

If a particular planner parameter has not been provided, default values are
loaded. Please see <> for the default planner configurations and a schema for
the configuration file.
--
- type : Sequence
    - type : Unique
        - name : RRT
        - range : 1
    - type : Unique
        - name : Snap
    - type : Unique
        - name : BITstar
        - batchSize : 100
--
*/
std::vector<aikido::planner::PlannerPtr> plannerFromFile(
    const aikido::statespace::ConstStateSpacePtr stateSpace,
    const std::string& configurationFile);

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_PLANNERFROMFILE_HPP_
