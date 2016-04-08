#include <aikido/ompl/DARTStateSampler.hpp>
#include <aikido/ompl/DARTGeometricStateSpace.hpp>

namespace aikido {
    namespace ompl_bindings {

        DARTStateSampler::DARTStateSampler(const ompl::base::StateSpace* _space,
                                           const aikido::constraint::SampleableConstraintPtr &_constraint) 
            : ompl::base::StateSampler(_space), mConstraint(_constraint)
        {

        }

        /// Sample a state uniformly from the space
        void DARTStateSampler::sampleUniform(ompl::base::State* _state)
        {
            auto state = static_cast<DARTGeometricStateSpace::StateType*>(_state);
            auto generator = mConstraint->createSampleGenerator();

            if(generator->canSample()){
                bool valid = generator->sample(state->mState);
            }

            // TODO: What to do if unable to sample?                            
        }

        /// Sample a state near another, within specified distance
        void DARTStateSampler::sampleUniformNear(ompl::base::State* _state, 
                                                 const ompl::base::State* _near,
                                                 const double _distance)
        {

        }

        /// Sample a state using a Gaussian distribution with given mean and standard deviation
        void DARTStateSampler::sampleGaussian(ompl::base::State* _state,
                                              const ompl::base::State* _mean,
                                              const double _stdDev) 
        {

        }

    }
}
