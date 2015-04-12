#ifndef R3_SKELETON_H_
#define R3_SKELETON_H_

#include <string>

namespace r3 {

class Skeleton {
public:
    Skeleton();
    virtual ~Skeleton();

    std::string name() const;
};

} // namespace r3

#endif
