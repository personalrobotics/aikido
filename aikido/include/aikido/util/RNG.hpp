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

  RNGWrapper(T _rng)
    : mRng(_rng)
  {
  }

  RNGWrapper(result_type _seed)
    : mRng(_seed)
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

  result_type min() const override
  {
    return mRng.min();
  }

  result_type max() const override
  {
    return mRng.max();
  }

  result_type operator()() override
  {
    return mRng();
  }
  
  std::unique_ptr<RNG> clone() const override
  {
    return std::unique_ptr<RNGWrapper>(new RNGWrapper(mRng));
  }

private:
  T mRng;
};

} // util
} // aikido

#endif // AIKIDO_UTIL_RNG_H_
