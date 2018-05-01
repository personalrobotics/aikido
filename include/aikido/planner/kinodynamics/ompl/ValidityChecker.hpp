#ifndef VALIDITYCHECK_H_
#define VALIDITYCHECK_H_

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <ompl/base/StateValidityChecker.h>
#include <aikido/planner/kinodynamics/ompl/OmplHelpers.hpp>

class Obstacle
{
public:
    Obstacle() {};
    virtual bool isValid(const ompl::base::State * state) const = 0;

    Eigen::VectorXd getPosVec(const ompl::base::State* state) const
    {
        const ompl::base::RealVectorStateSpace::StateType *state_rv =
                state->as<ompl::base::RealVectorStateSpace::StateType>();
        Eigen::VectorXd vec(param.dimensions/2);
        for(uint i=0; i<param.dimensions/2;i++)
        {
            vec[i] = state_rv->values[i*2];
        }
        return vec;
    }
};

class NSphere : public Obstacle
{
public:
    NSphere(unsigned int dimension, Eigen::VectorXd center, double radius)
        : dimension_(dimension), center_(center), radius_(radius)
    {
    }

    bool isValid(const ompl::base::State *state) const;

    unsigned int dimension_;
    Eigen::VectorXd center_;
    double radius_;
    ompl::base::SpaceInformationPtr si_;
};

class Hypercube : public Obstacle
{
public:
    Hypercube(unsigned int dimension, Eigen::VectorXd center, Eigen::VectorXd radius)
          : dimension_(dimension), center_(center), radius_(radius)
    {
    }

    bool isValid(const ompl::base::State *state) const;

    unsigned int dimension_;
    Eigen::VectorXd center_;
    Eigen::VectorXd radius_;
    ompl::base::SpaceInformationPtr si_;
};

class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }
    virtual ~ValidityChecker();

    bool addNSphereObstacle(Eigen::VectorXd center, double radius);
    bool addHypercubeObstacle(Eigen::VectorXd center, Eigen::VectorXd radius);
    bool isValid(const ompl::base::State *state) const;

    std::vector<Obstacle*> obstacles_;
};


#endif // VALIDITYCHECK_H_
