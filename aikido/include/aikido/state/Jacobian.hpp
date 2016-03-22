#ifndef AIKIDO_STATE_JACOBIAN_H
#define AIKIDO_STATE_JACOBIAN_H

#include "State.hpp"

namespace aikido {
namespace state{

template <class T>
class Jacobian{

};

class RealVectorJacobian: public Jacobian<RealVectorState>
{

};

}
}

#endif
