#include <aikido/ompl/DARTGeometricStateSpace.hpp>

namespace aikido {
    namespace ompl_bindings {

        DARTGeometricStateSpace::DARTGeometricStateSpace(const aikido::statespace::StateSpacePtr &_sspace)
                : mStateSpace(_sspace) {
            }

        /// Get the dimension of the space (not the dimension of the surrounding ambient space)
        unsigned int DARTGeometricStateSpace::getDimension() const {
            return mStateSpace->getDimension();
        }

        /// Get the maximum value a call to distance() can return (or an upper bound). 
        /// For unbounded state spaces, this function can return infinity.
        double DARTGeometricStateSpace::getMaximumExtent() const {
            return mStateSpace->getMaximumExtent();
        }

        /// Get a measure of the space (this can be thought of as a generalization of volume) 
        double DARTGeometricStateSpace::getMeasure() const {
            return mStateSpace->getMeasure();
        }

        /// Bring the state within the bounds of the state space. 
        /// For unbounded spaces this function can be a no-op.
        void DARTGeometricStateSpace::enforceBounds(ompl::base::State* _state) const {
//            auto st = static_cast<StateType*>(_state);
//            mStateSpace->enforceBounds(st->mState);
        }

        /// Check if a state is inside the bounding box. 
        /// For unbounded spaces this function can always return true.
        bool DARTGeometricStateSpace::satisfiesBounds(const ompl::base::State* _state) const {
//            auto st = static_cast<const StateType*>(_state);
//            return mStateSpace->satisfiesBounds(st->mState);
        }

        /// Copy a state to another.
        void DARTGeometricStateSpace::copyState(ompl::base::State* _destination, 
                                                const ompl::base::State* _source) const {
            
            auto dst = static_cast<StateType*>(_destination);
            auto sst = static_cast<const StateType*>(_source);
            mStateSpace->copyState(dst->mState, sst->mState);
        }

        /// Computes distance between two states. This function satisfies 
        /// the properties of a metric if isMetricSpace() is true, and its 
        /// return value will always be between 0 and getMaximumExtent()
        double DARTGeometricStateSpace::distance(const ompl::base::State* _state1,
                                                 const ompl::base::State* _state2) const {
            auto st1 = static_cast<const StateType*>(_state1);
            auto st2 = static_cast<const StateType*>(_state2);
            return mStateSpace->distance(st1->mState, st2->mState);
        }
            
        /// Check state equality
        bool DARTGeometricStateSpace::equalStates(const ompl::base::State* _state1,
                                                  const ompl::base::State* _state2) const {
            auto st1 = static_cast<const StateType*>(_state1);
            auto st2 = static_cast<const StateType*>(_state2);
            return mStateSpace->equalStates(st1->mState, st2->mState);
        }

        /// Computes the state that lies at time t in [0, 1] on the segment 
        /// that connects from state to to state. The memory location of state 
        /// is not required to be different from the memory of either from or to. 
        void DARTGeometricStateSpace::interpolate(const ompl::base::State* _from,
                                                  const ompl::base::State* _to,
                                                  const double _t,
                                                  ompl::base::State* _state) const {
            auto fromState = static_cast<const StateType*>(_from);
            auto toState = static_cast<const StateType*>(_to);
            auto iState = static_cast<StateType*>(_state);
            
            return mStateSpace->interpolate(fromState->mState,
                                            toState->mState,
                                            _t, iState->mState);
        }

        /// Allocate an instance of the state sampler for this space. 
        /// This sampler will be allocated with the sampler allocator that 
        /// was previously specified by setStateSamplerAllocator() or, 
        /// if no sampler allocator was specified, allocDefaultStateSampler() is called.
        ompl::base::StateSamplerPtr DARTGeometricStateSpace::allocDefaultStateSampler() const {
            // TODO
        }
        
        /// Allocate a state that can store a point in the described space
        ompl::base::State* DARTGeometricStateSpace::allocState() const {
            auto ast = mStateSpace->allocateState();
            return new StateType(ast);
        }
        
        /// Free the memory of the allocated state
        void DARTGeometricStateSpace::freeState(ompl::base::State* _state) const {
            auto st = static_cast<StateType*>(_state);
            mStateSpace->freeState(st->mState);
            delete st;
        }
    }
}
