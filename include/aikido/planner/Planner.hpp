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
  /// \param[in] rng RNG that planner uses. If nullptr, a default is created.
  explicit Planner(
      statespace::ConstStateSpacePtr stateSpace, common::RNG* rng = nullptr);

  /// Default destructor.
  virtual ~Planner() = default;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns RNG.
  common::RNG* getRng();

  /// Clones this planner.
  /// \param[in] rng RNG for the cloned planner to use. If nullptr,
  /// the default is cloned.
  virtual std::shared_ptr<Planner> clone(common::RNG* rng = nullptr) const = 0;

  /// Returns true if this planner can solve \c problem.
  virtual bool canSolve(const Problem& problem) const = 0;

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr)
      = 0;

  /// Abort planning if its running.
  /// Returns true if successfully stopped or not running a plan.
  virtual bool stopPlanning() = 0;

protected:
  /// State space associated with this planner.
  statespace::ConstStateSpacePtr mStateSpace;

  /// RNG the planner uses.
  std::unique_ptr<common::RNG> mRng;
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
