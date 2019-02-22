#include "aikido/perception/DetectedObject.hpp"

namespace aikido {
namespace perception {

//==============================================================================
template <typename T>
T DetectedObject::getInfoByKey(const std::string& key) const
{
  T value;
  try
  {
    value = mYamlNode[key].as<T>();
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[DetectedObject] Error in converting [" + key + "] field");
  }
  return value;
}

} // namespace perception
} // namespace aikido
