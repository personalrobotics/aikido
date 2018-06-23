#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <string>

#include "aikido/common/pointers.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(Planner)

/// Base class for a meta-planner
class Planner
{
public:
  class Result;

  /// Constructs from a state space.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  explicit Planner(
      statespace::ConstStateSpacePtr stateSpace,
      common::RNG* rng = nullptr);

  /// Default destructor.
  virtual ~Planner() = default;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns RNG.
  common::RNG* getRng() const;

  /// Returns true if this planner can solve \c problem.
  virtual bool canSolve(const Problem& problem) const = 0;

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr)
      = 0;

protected:
  /// State space associated with this planner.
  statespace::ConstStateSpacePtr mStateSpace;

  /// RNG the planner uses.
  common::RNG* mRng;
};

/// Base class for planning result of various planning problems.
class Planner::Result
{
public:
  /// Constructor.
  ///
  /// \param[in] message Planning result message.
  explicit Result(const std::string& message = "");

  /// Destructor.
  virtual ~Result() = default;

  /// Sets message.
  void setMessage(const std::string& message);

  /// Returns message.
  const std::string& getMessage() const;

protected:
  /// Message.
  std::string mMessage;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANNER_HPP_
