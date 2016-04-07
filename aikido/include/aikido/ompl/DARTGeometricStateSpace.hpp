#ifndef DART_GEOMETRIC_STATE_SPACE_H
#define DART_GEOMETRIC_STATE_SPACE_H

namespace aikido {
    namespace ompl {

        // Basic idea: Wrap the state space and associated state
        //  into a state space recognizable by OMPL

        /// Wraps an aikido StateSpace into a space recognized by OMPL
        class DARTGeometricStateSpace : public ompl::base::StateSpace {

        public:

            class StateType : public State {
            public:
                StateType(aikido::statespace::StateSpace::State* _st) : st(_st) {}

                aikido::statespace::StateSpace::State* mState;
            };
            
            DARTGeometricStateSpace(const aikido::statespace::StateSpacePtr &_sspace)
                : mStateSpace(_sspace) {
            }

            /// Get the dimension of the space (not the dimension of the surrounding ambient space)
            virtual unsigned int getDimension() const {
                return mStateSpace->getDimension();
            }

            /// Get the maximum value a call to distance() can return (or an upper bound). 
            /// For unbounded state spaces, this function can return infinity.
            virtual double getMaximumExtent() const {
                return mStateSpace->getMaximumExtent();
            }

            /// Get a measure of the space (this can be thought of as a generalization of volume) 
            virtual double getMeasure() const {
                return mStateSpace->getMeasure();
            }

            /// Bring the state within the bounds of the state space. 
            /// For unbounded spaces this function can be a no-op.
            virtual void enforceBounds(ompl::base::State* _state) const {
                StateType* st = static_cast<StateType*>(_state);
                mStateSpace->enforceBounds(st->mState);
            }

            /// Check if a state is inside the bounding box. 
            /// For unbounded spaces this function can always return true.
            virtual bool satisfiesBounds(const ompl::base::State* _state) const {
                StateType* st = static_cast<StateType*>(_state);
                return mStateSpace->satisfiesBounds(st->mState);
            }

            /// Copy a state to another.
            virtual void copyState(ompl::base::State* _destination, 
                                   const ompl::base::State* _source) const {
                
                StateType* dst = static_cast<StateType*>(_destination);
                StateType* sst = static_cast<StateType*>(_source);
                mStateSpace->copyState(dst.mState, sst.mState);
            }

            /// Computes distance between two states. This function satisfies 
            /// the properties of a metric if isMetricSpace() is true, and its 
            /// return value will always be between 0 and getMaximumExtent()
            virtual double distance(const ompl::base::State* _state1,
                                    const ompl::base::State* _state2) const {
                StateType* st1 = static_cast<StateType*>(_state1);
                StateType* st2 = static_cast<StateType*>(_state2);
                return mStateSpace->distance(st1.mState, st2.mState);
            }
            
            /// Check state equality
            virtual bool equalStates(const ompl::base::State* _state1,
                                       const ompl::base::State* _state2) const {
                StateType* st1 = static_cast<StateType*>(_state1);
                StateType* st2 = static_cast<StateType*>(_state2);
                return mStateSpace->equalStates(st1.mState, st2.mState);
            }

            /// Computes the state that lies at time t in [0, 1] on the segment 
            /// that connects from state to to state. The memory location of state 
            /// is not required to be different from the memory of either from or to. 
            virtual void interpolate(const ompl::base::State* _from,
                                     const ompl::base::State* _to,
                                     const double _t,
                                     ompl::base::State* _state) const {
                const StateType* fromState = static_cast<const StateType*>(_state1);
                const StateType* toState = static_cast<const StateType*>(_state2);
                StateType* iState = static_cast<StateType*>(_state);
                
                return mStateSpace->interpolate(fromState.mState,
                                                toState.mState,
                                                _t, iState.mState);
            }

            /// Allocate an instance of the state sampler for this space. 
            /// This sampler will be allocated with the sampler allocator that 
            /// was previously specified by setStateSamplerAllocator() or, 
            /// if no sampler allocator was specified, allocDefaultStateSampler() is called.
            virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const {
                // TODO
            }
                
            /// Allocate a state that can store a point in the described space
            virtual ompl::base::State* allocState() const {
                aikido::statespace::StateSpace::State* ast =
                    mStateSpace->allocateState();
                return new StateType(ast);
            }
            
            /// Free the memory of the allocated state
            virtual void freeState(ompl::base::State* _state) const {
                StateType* st = static_cast<StateType*>(_state);
                mStateSpace->freeState(st.st);
                delete st;
            }

        private:
            aikido::statespace::StateSpacePtr mStateSpace;

        };
    }
}
#endif
