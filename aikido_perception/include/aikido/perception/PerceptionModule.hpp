#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

//Abstract Class for perception modules.

namespace aikido {
namespace perception{

class PerceptionModule
{
public:

  virtual ~PerceptionModule() = default;


  //THE SIGNATURE MIGHT DIFFER FOR OTHER PERCEPTION METHODS - NEED TO ADDRESS
  virtual void detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double timeout) = 0;

};
}
}

#endif //AIKIDO_PERCEPTION_PERCEPTIONMODULE_H