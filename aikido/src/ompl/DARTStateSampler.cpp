#include <aikido/ompl/DARTStateSampler.hpp>
#include <aikido/ompl/DARTGeometricStateSpace.hpp>

namespace aikido {
    namespace ompl_bindings {

        DARTStateSampler::DARTStateSampler(const ompl::base::StateSpace* _space,
                                           std::unique_ptr<aikido::constraint::SampleGenerator> _generator) 
            : ompl::base::StateSampler(_space), mGenerator(std::move(_generator))
        {

        }

        /// Sample a state uniformly from the space
        void DARTStateSampler::sampleUniform(ompl::base::State* _state)
        {
            auto state = static_cast<DARTGeometricStateSpace::StateType*>(_state);

            bool valid = false;
            if(mGenerator->canSample()){
                valid = mGenerator->sample(state->mState);
            }

            if(!valid){
                throw std::domain_error("Failed to generate valid sample.");
            }
        }

        /// Sample a state near another, within specified distance
        void DARTStateSampler::sampleUniformNear(ompl::base::State* _state, 
                                                 const ompl::base::State* _near,
                                                 const double _distance)
        {
            throw std::runtime_error("sampleUniformNear not implemented.");
        }

        /// Sample a state using a Gaussian distribution with given mean and standard deviation
        void DARTStateSampler::sampleGaussian(ompl::base::State* _state,
                                              const ompl::base::State* _mean,
                                              const double _stdDev) 
        {
            throw std::runtime_error("sampleGaussian not implemented");
        }

    }
}
