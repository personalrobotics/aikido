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
  , mIncludeStartPoint(includeStartpoint)
  , mIncludeEndPoint(includeEndpoint)
  , mStartPoint(startPoint)
  , mEndPoint(endPoint)
{
  if (mStepSize > 0.0)
  {
    if (mStartPoint > mEndPoint)
    {
      throw std::runtime_error(
          "Start point is larger than end point when step size is positive");
    }
  }
  else if (mStepSize < 0.0)
  {
    if (mStartPoint < mEndPoint)
    {
      throw std::runtime_error(
          "Start point is smaller than end point when step size is negative");
    }
  }
  else
  {
    throw std::runtime_error("Step size is zero");
  }

  updateLength();
}

//==============================================================================
double StepSequence::operator[](std::size_t n) const
{
  const std::size_t length = getLength();
  if (n >= length)
    throw std::out_of_range("Invalid index.");

  if (!mIncludeStartPoint)
    ++n;

  const double val = mStartPoint + mStepSize * n;
  if (mStepSize > 0.0)
  {
    if (val > mEndPoint)
    {
      if (mIncludeEndPoint)
        return mEndPoint;
      else
        throw std::out_of_range("Indexed maximum intenger.");
    }
  }
  else
  {
    if (val < mEndPoint)
    {
      if (mIncludeEndPoint)
        return mEndPoint;
      else
        throw std::out_of_range("Indexed maximum intenger.");
    }
  }

  return val;
}

//==============================================================================
StepSequence::const_iterator StepSequence::begin() const
{
  StepSequence::const_iterator itr(*this, 0u);
  return itr;
}

//==============================================================================
StepSequence::const_iterator StepSequence::end() const
{
  StepSequence::const_iterator itr(*this, getLength());
  return itr;
}

//==============================================================================
std::size_t StepSequence::getLength() const
{
  return mNumSteps;
}

//==============================================================================
void StepSequence::updateLength()
{
  const double stepRatio = (mEndPoint - mStartPoint) / mStepSize;
  const double floorStepRatio = std::floor(stepRatio);

  mNumSteps = static_cast<std::size_t>(floorStepRatio);

  if (mIncludeStartPoint)
    ++mNumSteps;

  double endPivot = mStartPoint + floorStepRatio * mStepSize;

  if (mEndPoint > endPivot)
  {
    if (mIncludeEndPoint)
      ++mNumSteps;
  }
  else
  {
    if (!mIncludeEndPoint && mNumSteps > 0)
      --mNumSteps;
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
  if (mStep == mSeq.getLength())
    return;

  ++mStep;

  if (mStep == mSeq.getLength())
    return;

  mValue = mSeq[mStep];
}

//==============================================================================
bool StepSequence::const_iterator::equal(
    const StepSequence::const_iterator& other) const
{
  return other.mStep == mStep && &other.mSeq == &mSeq;
}

//==============================================================================
StepSequence::const_iterator::const_iterator(
    const StepSequence& seq, std::size_t step)
  : mSeq(seq), mStep(step)
{
  if (mStep < mSeq.getLength())
    mValue = mSeq[mStep];
}

} // namespace commons
} // namespace aikido
