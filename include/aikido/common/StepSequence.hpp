#ifndef AIKIDO_COMMON_STEPSEQUENCE_HPP_
#define AIKIDO_COMMON_STEPSEQUENCE_HPP_

#include <cassert>
#include <limits>
#include <tuple>

#include <boost/iterator/iterator_facade.hpp>

namespace aikido {
namespace common {

/// An iterator that returns a sequence of numbers between start point and end
/// point stepping at a fixed stepsize.
class StepSequence
{
public:
  class const_iterator;

  /// Constructor.
  ///
  /// \param stepSize Step size increments from the start point to the end
  /// point.
  /// \param includeStartpoint If includeStartpoint is true then the start point
  /// in the sequence will be the start point; else the start point in the
  /// sequence will be the start point plus the stepSize (if it is larger than
  /// the end point, it will be the end point.
  /// \param includeEndpoint If includeEndpoint is true then the final point in
  /// the sequence will be the end point, even if it is at less than stepSize
  /// from the second to last point.
  /// \param startPoint The start point that defines the sequence.
  /// \param endPoint The end point that defines the sequence.
  StepSequence(
      double stepSize,
      bool includeStartpoint = true,
      bool includeEndpoint = true,
      double startPoint = 0.0,
      double endPoint = 1.0);

  /// Constructs StepSequence in Matlab's linspace() style.
  StepSequence(
      double startPoint,
      double endPoint,
      std::size_t numSteps,
      bool includeEndpoint = true);

  /// Returns an iterator to the first element of the sequence.
  ///
  /// \return Iterator to the first element of the sequence.
  const_iterator begin() const;

  /// Returns an iterator to the element following the last element of the
  /// sequence.
  ///
  /// \return Iterator followin the last element of the sequence.
  const_iterator end() const;

  /// Returns the \c n-th element of the sequence.
  ///
  /// \return Element in the sequence.
  double operator[](std::size_t n) const;

  /// Returns the total length of sequence.
  ///
  /// \return Non-negative number of the tatal length of sequence.
  std::size_t getLength() const;

private:
  /// Computes the total length of sequence given step size. This is only called
  /// in the contructor.
  void updateNumSteps();

  /// Computes the step size of sequence given number of steps. This is only
  /// called in the contructor.
  void updateStepSize();

  /// Step size increments from the start point to the end point.
  double mStepSize;

  /// Whether the start point in the sequence will be the start point.
  const bool mIncludeStartPoint;

  /// Whether the end point in the sequence will be the end point.
  const bool mIncludeEndPoint;

  /// The start point that defines the sequence.
  const double mStartPoint;

  /// The end point that defines the sequence.
  const double mEndPoint;

  /// The total length of sequence.
  std::size_t mNumSteps;
};

class StepSequence::const_iterator
  : public boost::iterator_facade<
        StepSequence::const_iterator,
        double,
        boost::forward_traversal_tag,
        double>
{
public:
  /// Dereference implementation for boost::iterator_facade.
  double dereference() const;

  /// Increment implementation for boost::iterator_facade.
  void increment();

  /// Equal implementation for boost::iterator_facade.
  ///
  /// \return True if two iterators are at the same point in the sequence.
  bool equal(const StepSequence::const_iterator& other) const;

private:
  friend class StepSequence;

  /// Private constructor that should always be constructed from
  /// StepSequence::begin().
  const_iterator(const StepSequence& seq, std::size_t step);

  /// StepSequence associated with this iterator.
  const StepSequence& mSeq;

  /// Current step number.
  std::size_t mStep;

  /// Value of the current step.
  double mValue;
};

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_STEPSEQUENCE_HPP_
