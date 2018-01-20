#include <aikido/common/StepSequence.hpp>

#include <cmath>
#include <iostream>

namespace aikido {
namespace common {

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
  int endIdx = getMaxSteps();
  if(this->mIncludeEndpoints == false)
  {
    endIdx -= 1;
  }
  StepSequence::const_iterator itr(this, endIdx);
  if(this->mIncludeEndpoints == false)
  {
    itr.mStep = endIdx + 1;
  }
  return itr;
}

//==============================================================================
int StepSequence::getMaxSteps() const
{
  int numSteps = (mEndPoint - mStartPoint) / mStepSize;

  if (mEndPoint - (mStartPoint + mStepSize * numSteps) > 1e-7)
  {
    numSteps++;
  }
  return numSteps + mIncludeEndpoints;
}

//==============================================================================
double StepSequence::const_iterator::dereference() const
{
  return mValue;
}

//==============================================================================
void StepSequence::const_iterator::increment()
{
  int maxSteps = mSeq->getMaxSteps();
  mStep = std::min(maxSteps, mStep + 1);
  if (mSeq->mIncludeEndpoints==false && mStep > maxSteps-1)
  {
    mValue = (*mSeq)[mStep-1];
    return;
  }
  mValue = (*mSeq)[mStep];
}

//==============================================================================
bool StepSequence::const_iterator::equal(
    const StepSequence::const_iterator& other) const
{
  return other.mStep == mStep && other.mSeq == mSeq;
}

//==============================================================================
StepSequence::const_iterator::const_iterator(StepSequence* seq, int step)
  : mSeq(seq), mStep(step)
{
  mValue = (*mSeq)[mStep];
}

} // namespace commons
} // namespace aikido
