#ifndef AIKIDO_UTIL_RNG_H_
#define AIKIDO_UTIL_RNG_H_

#include <ctime>
#include <memory>
#include <stdint.h>

namespace aikido {
namespace util {

class RNG
{
public:
  typedef uint32_t result_type;

  virtual result_type min() const = 0;
  virtual result_type max() const = 0;
  virtual result_type operator()() = 0;
  virtual std::unique_ptr<RNG> clone() const = 0;

};

template <class T>
class RNGWrapper : virtual public RNG
{
public:

  RNGWrapper()
    : mRng(std::time(0))
  {
  }

  RNGWrapper(result_type _seed)
    : mRng(_seed)
    , mSeed(_seed)
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
  
  virtual std::unique_ptr<RNG> clone() const override
  {
    return std::unique_ptr<RNG>(new RNGWrapper(mSeed));
  }

private:
  T mRng;
  result_type mSeed;
};

} // util
} // aikido

#endif // AIKIDO_UTIL_RNG_H_
