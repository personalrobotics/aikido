#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

namespace aikido {
namespace perception{

class PerceptionModule
{
public:
  PerceptionModule();
  virtual ~PerceptionModule()
  {
  }

  virtual void DetectObject();
  virtual void DetectObjects();

};
}
}

#endif //AIKIDO_PERCEPTION_PERCEPTIONMODULE_H