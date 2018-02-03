#ifndef AIKIDO_COMMON_STEPSEQUENCE_HPP_
#define AIKIDO_COMMON_STEPSEQUENCE_HPP_

#include <cassert>
#include <limits>
#include <tuple>
#include <boost/iterator/iterator_facade.hpp>

namespace aikido {
namespace common {

/// An iterator that returns a sequence of numbers between 0 and 1 stepping at a
/// fixed stepsize
class StepSequence
{
public:
  class const_iterator;

  /// Constructor
  /// \param stepSize step size increments from the start point to the end
  /// point.
  /// \param includeStartpoint If includeStartpoint is true then the start point
  /// in the sequence will be
  /// the start point; else the start point in the sequence will be the start
  /// point
  /// plus the stepSize (if it is larger than the end point, it will be the end
  /// point.
  /// \param includeEndpoint If includeEndpoint is true then the final point in
  /// the sequence
  /// will be the end point, even if it is at less than stepSize from the second
  /// to last point.
  /// \param startPoint the start point that defines the sequence
  /// \param endPoint the end point that defines the sequence
  StepSequence(
      double stepSize,
      bool includeStartpoint = true,
      bool includeEndpoint = true,
      double startPoint = 0.0,
      double endPoint = 1.0);

  /// Returns an iterator to the first element of the sequence.
  ///
  /// \return iterator to the first element of the sequence
  const_iterator begin();

  /// Returns an iterator to the element following the last element of the
  /// sequence.
  ///
  /// \return iterator followin the last element of the sequence
  const_iterator end();

  /// Returns the \c n-th element of the sequence
  ///
  /// \return element in the sequence
  double operator[](int n);

  /// Returns the total length of sequence.
  ///
  /// \return Non-negative number of the tatal length of sequence.
  std::size_t getLength() const;

private:
  double last() const;

  double mStepSize;
  double mStartPoint;
  double mEndPoint;
  bool mIncludeStartPoint;
  bool mIncludeEndpoint;
};

class StepSequence::const_iterator
    : public boost::iterator_facade<StepSequence::const_iterator,
                                    double,
                                    boost::forward_traversal_tag,
                                    double>
{
public:
  /// Dereference implementation for boost::iterator_facade
  double dereference() const;

  /// Increment implementation for boost::iterator_facade
  void increment();

  /// equal implementation for boost::iterator_facade
  /// \return True if two iterators are at the same point in the sequence
  bool equal(const StepSequence::const_iterator& other) const;

private:
  friend class StepSequence;

  /// Private constructor that should always be constructed from
  /// StepSequence::begin()
  const_iterator(StepSequence* seq);

  StepSequence* mSeq;
  std::size_t mStep;
  double mValue;
};

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_STEPSEQUENCE_HPP_
