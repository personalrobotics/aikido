#ifndef DART_STATE_SAMPLER_H
#define DART_STATE_SAMPLER_H

#include <ompl/base/StateSampler.h>
#include <aikido/constraint/Sampleable.hpp>

namespace aikido {
    namespace ompl_bindings {

        /// Wraps an aikido::constraint::SampleableConstraint in a ompl::base::StateSampler
        class DARTStateSampler : public ompl::base::StateSampler {

        public:

            DARTStateSampler(const ompl::base::StateSpace* _space,
                             const aikido::constraint::SampleableConstraintPtr &_constraint);

            /// Sample a state uniformly from the space
            virtual void sampleUniform(ompl::base::State* _state) = 0;

            /// Sample a state near another, within specified distance
            virtual void sampleUniformNear(ompl::base::State* _state, 
                                           const ompl::base::State* _near,
                                           const double distance);

            /// Sample a state using a Gaussian distribution with given mean and standard deviation
            virtual void sampleGaussian(ompl::base::State* _state,
                                        const ompl::base::State* _mean,
                                        const double stdDev);

        private:
            aikido::constraint::SampleableConstraintPtr mConstraint;

        };
    }
}
#endif
