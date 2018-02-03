#include <aikido/common/StepSequence.hpp>

#include <cmath>
#include <iostream>

namespace aikido {
namespace common {

constexpr double precision = 1e-7;

//==============================================================================
StepSequence::StepSequence(
    double stepSize,
    bool includeStartpoint,
    bool includeEndpoint,
    double startPoint,
    double endPoint)
  : mStepSize(stepSize)
  , mStartPoint(startPoint)
  , mEndPoint(endPoint)
  , mIncludeStartPoint(includeStartpoint)
  , mIncludeEndpoint(includeEndpoint)
{
  if (mStartPoint > mEndPoint)
    throw std::runtime_error("Start point is larger than end point");
}

//==============================================================================
double StepSequence::operator[](int n)
{
  int length = getLength();
  if (n < 0 || n >= length)
    throw std::out_of_range("Invalid index.");

  if (!mIncludeStartPoint)
  {
    ++n;
  }
  double val = mStartPoint + mStepSize * n;
  if (val > mEndPoint)
  {
    if (mIncludeEndpoint)
      return mEndPoint;
    else
      throw std::out_of_range("Indexed maximum intenger.");
  }
  return val;
}

//==============================================================================
StepSequence::const_iterator StepSequence::begin()
{
  StepSequence::const_iterator itr(this);
  return itr;
}

//==============================================================================
StepSequence::const_iterator StepSequence::end()
{
  StepSequence::const_iterator itr(this);
  itr.mStep = getLength();
  itr.mValue = last();
  return itr;
}

//==============================================================================
std::size_t StepSequence::getLength() const
{
  double stepRatio = (mEndPoint - mStartPoint) / mStepSize;
  double floorStepRatio = floor(stepRatio);
  std::size_t stepNum = static_cast<std::size_t>(floorStepRatio);
  if (mIncludeStartPoint)
  {
    ++stepNum;
  }
  double endPivot = mStartPoint + floorStepRatio * mStepSize;
  if (mEndPoint > endPivot)
  {
    if (mIncludeEndpoint)
      ++stepNum;
  }
  else
  {
    if (!mIncludeEndpoint && stepNum > 0)
      --stepNum;
  }

  return stepNum;
}

//==============================================================================
double StepSequence::last() const
{
  if (mIncludeEndpoint)
    return mEndPoint;
  std::size_t length = getLength();
  if (length == 0)
    return mStartPoint;
  return mStartPoint + mStepSize * (length - 1);
}

//==============================================================================
double StepSequence::const_iterator::dereference() const
{
  return mValue;
}

//==============================================================================
void StepSequence::const_iterator::increment()
{
  ++mStep;
  if (mStep >= mSeq->getLength())
  {
    mValue = mSeq->last();
  }
  else
  {
    mValue = (*mSeq)[mStep];
  }
}

//==============================================================================
bool StepSequence::const_iterator::equal(
    const StepSequence::const_iterator& other) const
{
  return other.mStep == mStep && other.mSeq == mSeq;
}

//==============================================================================
StepSequence::const_iterator::const_iterator(StepSequence* seq)
  : mSeq(seq), mStep(0), mValue(0.0)
{
  assert(seq);
  mValue = (*mSeq)[mStep];
}

} // namespace commons
} // namespace aikido
