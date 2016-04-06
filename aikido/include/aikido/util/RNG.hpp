#ifndef AIKIDO_UTIL_RNG_H_
#define AIKIDO_UTIL_RNG_H_

#include <ctime>
#include <memory>
#include <random>
#include <cstdint>

namespace aikido {
namespace util {

class RNG
{
public:
  using result_type = std::uint32_t;

  virtual ~RNG() = default;

  static constexpr result_type min() { return 0; }
  static constexpr result_type max() { return 0xFFFFFFFF; }

  virtual result_type operator()() = 0;
  virtual std::unique_ptr<RNG> clone() const = 0;

};

template <class T>
class RNGWrapper : virtual public RNG
{
public:
  using RNG::result_type;
  using engine_type = std::independent_bits_engine<T, 32, result_type>;

  RNGWrapper() = default;

  explicit RNGWrapper(const T& _rng)
    : mRng(_rng)
  {
  }

  virtual ~RNGWrapper() = default;

  T& rng()
  {
    return mRng.base();
  }

  const T& rng() const
  {
    return mRng.base();
  }

  result_type operator()() override
  {
    return mRng();
  }
  
  std::unique_ptr<RNG> clone() const override
  {
    return std::unique_ptr<RNGWrapper>(new RNGWrapper(mRng.base()));
  }

private:
  engine_type mRng;
};

} // util
} // aikido

#endif // AIKIDO_UTIL_RNG_H_
