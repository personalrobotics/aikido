#include <aikido/util/StepSequence.hpp>

#include <cmath>
#include <iostream>

namespace aikido {
namespace util {

//==============================================================================
StepSequence::StepSequence(
    double _stepSize,
    bool _includeEndpoints,
    double _startPoint,
    double _endPoint)
  : mStepSize(_stepSize)
  , mStartPoint(_startPoint)
  , mEndPoint(_endPoint)
  , mIncludeEndpoints(_includeEndpoints)
{
  // Do nothing
}

//==============================================================================
double StepSequence::operator[](int n)
{
  double val = mStartPoint + mStepSize * n;
  if (val > mEndPoint)
  {
    if (mIncludeEndpoints)
      return mEndPoint;
    else
      throw std::out_of_range("Indexed maximum intenger.");
  }
  else if (val < mStartPoint)
  {
    if (mIncludeEndpoints)
      return mStartPoint;
    else
      throw std::out_of_range("Invalid index.");
  }
  return val;
}

//==============================================================================
StepSequence::const_iterator StepSequence::begin()
{
  StepSequence::const_iterator itr(this, 0);
  return itr;
}

//==============================================================================
StepSequence::const_iterator StepSequence::end()
{
  StepSequence::const_iterator itr(this, getMaxSteps());
  return itr;
}

//==============================================================================
int StepSequence::getMaxSteps() const
{
  int numSteps = (mEndPoint - mStartPoint) / mStepSize;
  if (!mIncludeEndpoints)
  {
    // Return numSteps + 1 for the start
    return numSteps + 1;
  }
  else
  {
    if (fabs(mStartPoint + mStepSize * numSteps - mEndPoint) < 1e-7)
    {
      // Return numSteps + 1 for the start (endpt already included)
      return numSteps + 1;
    }
    else
    {
      // Return numSteps + 1 for the start + 1 for the end
      return numSteps + 2;
    }
  }
}

//==============================================================================
double StepSequence::const_iterator::dereference() const
{
  return mValue;
}

//==============================================================================
void StepSequence::const_iterator::increment()
{
  mStep = std::min(mSeq->getMaxSteps(), mStep + 1);
  mValue = (*mSeq)[mStep];
}

//==============================================================================
bool StepSequence::const_iterator::equal(
    const StepSequence::const_iterator& other) const
{
  return other.mStep == mStep && other.mSeq == mSeq;
}

} // namespace utils
} // namespace aikido
