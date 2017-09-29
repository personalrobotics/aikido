#include <aikido/util/RNG.hpp>

namespace aikido {
namespace util {

//==============================================================================
// This namespace-scoped definition is required to enable odr-use.
constexpr std::size_t RNG::NUM_BITS;

//==============================================================================
std::vector<std::unique_ptr<util::RNG>> cloneRNGsFrom(
    RNG& _engine, size_t _numOutputs, size_t _numSeeds)
{
  // Use the input RNG to create an initial batch of seeds.
  std::vector<util::RNG::result_type> initialSeeds;
  initialSeeds.reserve(_numSeeds);

  for (size_t iseed = 0; iseed < _numSeeds; ++iseed)
    initialSeeds.emplace_back(_engine());

  // Use seed_seq to improve the quality of our seeds.
  std::seed_seq seqSeeds(initialSeeds.begin(), initialSeeds.end());
  std::vector<util::RNG::result_type> improvedSeeds(_numOutputs);
  seqSeeds.generate(std::begin(improvedSeeds), std::end(improvedSeeds));

  // Create the random number generators of the same type as the input _engine.
  std::vector<std::unique_ptr<util::RNG>> output;
  output.reserve(_numOutputs);

  for (auto improvedSeed : improvedSeeds)
    output.emplace_back(_engine.clone(improvedSeed));

  return output;
}

//==============================================================================
std::vector<std::unique_ptr<util::RNG>> cloneRNGFrom(
    RNG& _engine, size_t _numSeeds)
{
  return cloneRNGsFrom(_engine, 1, _numSeeds);
}

} // namespace util
} // namespace aikido
