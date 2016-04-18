#ifndef AIKIDO_VANDERCORPUT_HPP_
#define AIKIDO_VANDERCORPUT_HPP_

#include <boost/iterator/iterator_facade.hpp>
#include <cassert>
#include <limits>
#include <tuple>

namespace aikido
{
namespace util
{
using std::pair;

/// An iterator that returns a sequence of numbers between 0 and 1 stepping at a
/// fixed stepsize
class StepSequence
{
public:
  class const_iterator;

  /// Constructor - step_size should be between 0 and 1
  StepSequence(const double _stepSize, const bool _includeEndpoints = false);

  const_iterator begin();
  const_iterator end();

  pair<double, double> operator[](int n);

private:
  double mStepSize;
  bool mIncludeEndpoints;
};

class StepSequence::const_iterator
    : public boost::iterator_facade<StepSequence::const_iterator, const double,
                                    boost::forward_traversal_tag>
{
public:
  // Constructor
  StepSequence::const_iterator()
      : mStep(0)
  {
  }

  explicit StepSequence::const_iterator(int step)
      : mStep(step)
  {
  }

private:
  friend class boost::iterator_core_access;

  void increment() { mStep++; }

  bool equal(StepSequence::const_iterator const &other) const
  {
    return this->mStep == other.mStep;
  }

  double dereference() const
  {
    
  };

}  // util
}  // aikido
#endif
