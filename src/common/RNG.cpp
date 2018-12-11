#include <aikido/common/RNG.hpp>

namespace aikido {
namespace common {

//==============================================================================
// This namespace-scoped definition is required to enable odr-use.
constexpr std::size_t RNG::NUM_BITS;

//==============================================================================
std::vector<std::unique_ptr<common::RNG>> cloneRNGsFrom(
    RNG& _engine, std::size_t _numOutputs,
    std::size_t _numSeeds,
    bool fixedSeeds,
    int baseSeed
) {
  // Create the random number generators of the same type as the input _engine.
  std::vector<std::unique_ptr<common::RNG>> output;
  output.reserve(_numOutputs);

  if (fixedSeeds)
  {
    for (int curSeed = baseSeed; curSeed < baseSeed + _numOutputs; curSeed++)
      output.emplace_back(_engine.clone(curSeed));
  }
  else
  {
    // Use the input RNG to create an initial batch of seeds.
    std::vector<common::RNG::result_type> initialSeeds;
    initialSeeds.reserve(_numSeeds);

    for (std::size_t iseed = 0; iseed < _numSeeds; ++iseed)
      initialSeeds.emplace_back(_engine());

    // Use seed_seq to improve the quality of our seeds.
    std::seed_seq seqSeeds(initialSeeds.begin(), initialSeeds.end());
    std::vector<common::RNG::result_type> improvedSeeds(_numOutputs);
    seqSeeds.generate(std::begin(improvedSeeds), std::end(improvedSeeds));

    for (auto improvedSeed : improvedSeeds)
      output.emplace_back(_engine.clone(improvedSeed));
  }

  return output;
}

//==============================================================================
std::vector<std::unique_ptr<common::RNG>> cloneRNGFrom(
    RNG& _engine, std::size_t _numSeeds)
{
  return cloneRNGsFrom(_engine, 1, _numSeeds);
}

} // namespace common
} // namespace aikido
