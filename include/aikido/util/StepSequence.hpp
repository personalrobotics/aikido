#ifndef AIKIDO_UTIL_STEPSEQUENCE_HPP_
#define AIKIDO_UTIL_STEPSEQUENCE_HPP_

#include <cassert>
#include <limits>
#include <tuple>
#include <boost/iterator/iterator_facade.hpp>

namespace aikido {
namespace util {

/// An iterator that returns a sequence of numbers between 0 and 1 stepping at a
/// fixed stepsize
class StepSequence
{
public:
  class const_iterator;

  /// Constructor - step at _stepSize increments from the start point to the end
  /// point. If includeEndpoints is true then the final point in the sequence
  /// will be the end point, even if it is at less than stepSize from the second
  /// to last point.
  StepSequence(
      double _stepSize,
      bool _includeEndpoints = true,
      double _startPoint = 0.0,
      double _endPoint = 1.0);

  const_iterator begin();
  const_iterator end();

  double operator[](int n);

  /// Compute the maximum number of steps in the sequence
  int getMaxSteps() const;

private:
  double mStepSize;
  double mStartPoint;
  double mEndPoint;
  bool mIncludeEndpoints;
};

class StepSequence::const_iterator
    : public boost::iterator_facade<StepSequence::const_iterator,
                                    const double,
                                    boost::forward_traversal_tag>
{
public:
  /// Return the value of the iterator
  double const& dereference() const;

  /// Increment the sequence
  void increment();

  /// True if two iterators are at the same point in the sequence
  bool equal(const StepSequence::const_iterator& other) const;

private:
  friend class StepSequence;

  const_iterator(StepSequence* seq, int step) : mSeq(seq), mStep(step)
  {
    mValue = (*mSeq)[mStep];
  }

  StepSequence* mSeq;
  int mStep;
  double mValue;
};

} // namespace util
} // namespace aikido

#endif // AIKIDO_UTIL_STEPSEQUENCE_HPP_
