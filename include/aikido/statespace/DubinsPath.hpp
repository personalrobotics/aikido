#ifndef AIKIDO_COMMON_DUBINSPATH_HPP_
#define AIKIDO_COMMON_DUBINSPATH_HPP_

#include <aikido/statespace/SE2.hpp>

namespace aikido {
namespace common {

AIKIDO_DECLARE_POINTERS(AbstractDubinsPath)
AIKIDO_DECLARE_POINTERS(DudinsPath)

class AbstractDubinsPath{
public:
    virtual double length() = 0;
    virtual aikido::statespace::SE2::State interpolate(double alpha) = 0;
};

class DubinsPath : AbstractDubinsPath{
public:

    DubinsPath(aikido::statespace::SE2::State StateFrom,
               aikido::statespace::SE2::State StateTo,
               double turningRadius);
    
    double length() override;

    aikido::statespace::SE2::State interpolate(double alpha) override;

private:

    aikido::statespace::SE2::State mStateFrom;
    aikido::statespace::SE2::State mStateTo;
    double mTurningRadius;
    AbstractDubinsPathPtr mPath;
};

} //namespace common
} //namespace aikido

#endif