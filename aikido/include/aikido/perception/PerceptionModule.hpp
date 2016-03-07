#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

namespace aikido {
namespace perception{

class PerceptionModule
{
public:

  virtual ~PerceptionModule() = default;

  virtual void DetectObject() const = 0;

};
}
}

#endif //AIKIDO_PERCEPTION_PERCEPTIONMODULE_H