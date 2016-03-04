#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

namespace aikido {
namespace perception{

class PerceptionModule: 
{
public:
  PerceptionModule();
  virtual ~PerceptionModule()
  {
  }

  virtual void DetectObject(WHAT HERE?);
  virtual void DetectObjects(WHAT HERE?);

};
}
}