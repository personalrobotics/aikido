#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include <aikido/common/VanDerCorput.hpp>

namespace aikido {
namespace common {

using std::pair;

//==============================================================================
// Required for odr-use.
constexpr int VanDerCorput::BASE;
constexpr int VanDerCorput::MAX;

//==============================================================================
VanDerCorput::VanDerCorput(
    double span,
    bool includeStartpoint,
    bool includeEndpoint,
    double minResolution)
  : mSpan(span)
  , mIncludeStartpoint(includeStartpoint)
  , mIncludeEndpoint(includeEndpoint)
  , mMinResolution(minResolution)
{
  mMinResolution
      = std::max(mMinResolution, std::numeric_limits<double>::epsilon());
}

//==============================================================================
pair<double, double> VanDerCorput::operator[](int n) const
{
  pair<double, double> val_res;

  if (n == VanDerCorput::MAX)
  {
    throw std::out_of_range("Indexed maximum integer.");
  }

  if (mIncludeStartpoint && mIncludeEndpoint)
  {
    if (n == 0)
    {
      val_res.first = 0.0;
      val_res.second = mSpan;
    }
    else if (n == 1)
    {
      val_res.first = 1.0;
      val_res.second = mSpan;
    }
    else
    {
      val_res = computeVanDerCorput(n - 1);
    }
  }
  else if (mIncludeStartpoint && n == 0)
  {
    val_res.first = 0.0;
    val_res.second = mSpan;
  }
  else if (mIncludeEndpoint && n == 0)
  {
    val_res.first = 1.0;
    val_res.second = mSpan;
  }
  else
  {
    if (mIncludeStartpoint || mIncludeEndpoint)
    {
      val_res = computeVanDerCorput(n);
    }
    else
    {
      val_res = computeVanDerCorput(n + 1);
    }
  }

  val_res.first *= mSpan;
  return val_res;
}

//==============================================================================
std::size_t VanDerCorput::getLength() const
{
  return std::distance(begin(), end());
}

//==============================================================================
pair<double, double> VanDerCorput::computeVanDerCorput(int n) const
{
  // range: [1,int_max]
  double denom = 1;
  double resolution = 1;
  double ret = 0.0;

  // Treat Van Der Corput sequence like a binary tree.
  // Each node that completes a perfect tree
  // reduces the resolution by cutting the final remaining
  // segment of the last resolution size.
  // So to find the resolution...
  double power = std::ceil(std::log2(n + 1)) - 1; // calc height of tree
  double next_power
      = std::ceil(std::log2(n + 2)) - 1; // and height after next node is added.
  if (power == next_power)
  { // If next node does not start new level
    resolution = 1. / (std::pow(2, power)); // not yet perfect tree
  }
  else
  { // if next node does start new level
    resolution = 1. / (std::pow(2, power + 1)); // shrink resolution
  }

  while (n)
  {
    denom *= BASE;
    ret += (n % BASE) / denom;
    n /= BASE;
  }

  return std::make_pair(ret, resolution);
}

//==============================================================================
VanDerCorput::const_iterator VanDerCorput::begin() const
{
  VanDerCorput::const_iterator itr{this};
  return itr;
}

//==============================================================================
VanDerCorput::const_iterator VanDerCorput::end() const
{
  VanDerCorput::const_iterator itr{this};
  itr.mN = VanDerCorput::MAX;
  return itr;
}

//==============================================================================
VanDerCorput::const_iterator::const_iterator(const VanDerCorput* seq)
  : mSeq(seq), mN(-1), mFinalIter(false)
{
  assert(mSeq);
  increment();
}

//==============================================================================
double VanDerCorput::const_iterator::dereference() const
{
  return mCurr.first;
}

//==============================================================================
void VanDerCorput::const_iterator::increment()
{
  if (mFinalIter)
  {
    mN = VanDerCorput::MAX;
  }
  else
  {
    ++mN;
    mCurr = (*mSeq)[mN];
    if (mCurr.second <= mSeq->mMinResolution)
    {
      mFinalIter = true;
    }
  }
}

//==============================================================================
bool VanDerCorput::const_iterator::equal(
    const VanDerCorput::const_iterator& other) const
{
  return other.mN == mN && other.mSeq == mSeq;
}

} // namespace common
} // namespace aikido
