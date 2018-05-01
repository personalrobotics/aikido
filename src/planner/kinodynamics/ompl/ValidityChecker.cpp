#include <aikido/planner/kinodynamics/ompl/ValidityChecker.hpp>

bool NSphere::isValid(const ompl::base::State *state) const
{
    Eigen::VectorXd stateVec = getPosVec(state);
    Eigen::VectorXd delta = stateVec - center_;
    if( delta.norm() < radius_ )
    {
        return false;
    }
    return true;
}

bool Hypercube::isValid(const ompl::base::State *state) const
{
    Eigen::VectorXd stateVec = getPosVec(state);
    Eigen::VectorXd delta = stateVec - center_;
    for(uint i=0;i< param.dimensions/2; ++i)
    {
        if( std::abs(stateVec[i]-center_[i]) >= radius_[i] )
        {
            return true;
        }
    }
    return false;
}

ValidityChecker::~ValidityChecker()
{
    for(std::vector<Obstacle*>::iterator it = obstacles_.begin();
        it != obstacles_.end(); it++)
    {
        Obstacle* p_obs = (*it);
        delete p_obs;
        p_obs = NULL;
    }
    obstacles_.clear();
}

bool ValidityChecker::addNSphereObstacle(Eigen::VectorXd center, double radius)
{
    NSphere* pNS = new NSphere(param.dimensions, center, radius);
    obstacles_.push_back(pNS);
    return true;
}

bool ValidityChecker::addHypercubeObstacle(Eigen::VectorXd center, Eigen::VectorXd radius)
{
    Hypercube* pHC = new Hypercube(param.dimensions, center, radius);
    obstacles_.push_back(pHC);
    return true;
}

bool ValidityChecker::isValid(const ompl::base::State *state) const
{


    if( false == si_->satisfiesBounds( state ) )
    {
        return false;
    }

    for(std::vector<Obstacle*>::const_iterator it = obstacles_.begin();
        it != obstacles_.end(); ++it)
    {
        Obstacle* p_obs = (*it);
        if(false == p_obs->isValid(state))
        {
            return false;
        }
    }
    return true;
}
