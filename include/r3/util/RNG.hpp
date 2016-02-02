#ifndef R3_UTIL_RNG_H_
#define R3_UTIL_RNG_H_

#include <ctime>
#include <memory>
#include <stdint.h>

namespace r3 {
namespace util {

class RNG
{
public:
  typedef uint32_t result_type;
  typedef std::shared_ptr<RNG> Ptr;
  typedef std::shared_ptr<RNG const> ConstPtr;

  virtual result_type min() const = 0;
  virtual result_type max() const = 0;
  virtual result_type operator()() = 0;
};

template <class T>
class RNGWrapper : virtual public RNG
{
public:
  typedef std::shared_ptr<RNGWrapper> Ptr;
  typedef std::shared_ptr<RNGWrapper const> ConstPtr;

  RNGWrapper()
    : mRng(std::time(0))
  {
  }

  RNGWrapper(result_type seed)
    : mRng(seed)
  {
  }

  T &rng()
  {
    return mRng;
  }

  T const &rng() const
  {
    return mRng;
  }

  virtual result_type min() const
  {
    return mRng.min();
  }

  virtual result_type max() const
  {
    return mRng.max();
  }

  virtual result_type operator()()
  {
    return mRng();
  }

private:
  T mRng;
};

} // util
} // r3

#endif // R3_UTIL_RNG_H_
