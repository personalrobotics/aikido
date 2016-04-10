#ifndef DART_GEOMETRIC_STATE_SPACE_H
#define DART_GEOMETRIC_STATE_SPACE_H

#include <ompl/base/StateSpace.h>
#include <aikido/statespace/StateSpace.hpp>

namespace aikido {
    namespace ompl_bindings {

        /// Wraps an aikido StateSpace into a space recognized by OMPL
        class DARTGeometricStateSpace : public ompl::base::StateSpace {

        public:

            class StateType : public ompl::base::State {
            public:
                StateType(aikido::statespace::StateSpace::State* _st) : mState(_st) {}

                aikido::statespace::StateSpace::State* mState;
            };
            
            DARTGeometricStateSpace(const aikido::statespace::StateSpacePtr &_sspace);

            /// Get the dimension of the space (not the dimension of the surrounding ambient space)
            virtual unsigned int getDimension() const;

            /// Get the maximum value a call to distance() can return (or an upper bound). 
            /// For unbounded state spaces, this function can return infinity.
            virtual double getMaximumExtent() const;

            /// Get a measure of the space (this can be thought of as a generalization of volume) 
            virtual double getMeasure() const;

            /// Bring the state within the bounds of the state space. 
            /// For unbounded spaces this function can be a no-op.
            virtual void enforceBounds(ompl::base::State* _state) const;

            /// Check if a state is inside the bounding box. 
            /// For unbounded spaces this function can always return true.
            virtual bool satisfiesBounds(const ompl::base::State* _state) const;

            /// Copy a state to another.
            virtual void copyState(ompl::base::State* _destination, 
                                   const ompl::base::State* _source) const;

            /// Computes distance between two states. This function satisfies 
            /// the properties of a metric if isMetricSpace() is true, and its 
            /// return value will always be between 0 and getMaximumExtent()
            virtual double distance(const ompl::base::State* _state1,
                                    const ompl::base::State* _state2) const;
            
            /// Check state equality
            virtual bool equalStates(const ompl::base::State* _state1,
                                     const ompl::base::State* _state2) const;

            /// Computes the state that lies at time t in [0, 1] on the segment 
            /// that connects from state to to state. The memory location of state 
            /// is not required to be different from the memory of either from or to. 
            virtual void interpolate(const ompl::base::State* _from,
                                     const ompl::base::State* _to,
                                     const double _t,
                                     ompl::base::State* _state) const;

            /// Allocate an instance of the state sampler for this space. 
            /// This sampler will be allocated with the sampler allocator that 
            /// was previously specified by setStateSamplerAllocator() or, 
            /// if no sampler allocator was specified, allocDefaultStateSampler() is called.
            virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
                
            /// Allocate a state that can store a point in the described space
            virtual ompl::base::State* allocState() const;
            
            /// Free the memory of the allocated state
            virtual void freeState(ompl::base::State* _state) const;

        private:
            aikido::statespace::StateSpacePtr mStateSpace;

        };
    }
}
#endif
