#include <iostream>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <aikido/planner/kinodynamics/ompl/MyBITstar.h>
#include <aikido/planner/kinodynamics/dimt/Params.h>

using namespace ompl;
using namespace ompl::geometric;

// Our stuff


namespace ompl
{
namespace geometric
{


MyBITstar::MyBITstar(const ompl::base::SpaceInformationPtr &si) : BITstar(si)
{
    mode_ = RANDOM_SAMPLES;

}

void MyBITstar::initLogFile(std::string scenarioName, std::string samplerName, int id)
{
    std::stringstream ss;
    ss << scenarioName.c_str() << "_" << samplerName.c_str() << "_" << id << ".csv";
    out_.open(ss.str());

    std::cout << "SAVING FILE TO " << ss.str() << " = " << out_.is_open() << std::endl;
}

base::PlannerStatus MyBITstar::solve(const base::PlannerTerminationCondition &ptc)
{
    return BITstar::solve(ptc);
}


ompl::base::PlannerStatus MyBITstar::solve(double solveTime)
{
    if (solveTime < 1.0)
        return solve(ompl::base::timedPlannerTerminationCondition(solveTime));
    return solve(ompl::base::timedPlannerTerminationCondition(solveTime,
                                                              std::min(solveTime / 100.0, 0.1)));
}

ompl::base::PlannerStatus MyBITstar::solveAfterLoadingSamples(std::string filename, double solveTime)
{
    mode_ = LOAD_SAMPLES;

    /*
    if(nn_)
    {
        //nn_.reset(new NearestNeighborsLinear<Motion *>());
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    }*/

    sampleLoadStream_.open(filename.c_str(), std::ios::in);
    if (solveTime < 1.0)
        return solve(ompl::base::timedPlannerTerminationCondition(solveTime));
    return solve(ompl::base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
}

ompl::base::PlannerStatus MyBITstar::solveAndSaveSamples(std::string filename, double solveTime)
{
    mode_ = SAVE_SAMPLES;

    /*
    if(nn_)
    {
        //nn_.reset(new NearestNeighborsLinear<Motion *>());
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    }*/

    sampleSaveStream_.open(filename.c_str(), std::ios::out);
    if (solveTime < 1.0)
        return solve(ompl::base::timedPlannerTerminationCondition(solveTime));
    return solve(ompl::base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
}


bool MyBITstar::toState(std::string stateString, ompl::base::State* toState)
{
    if(toState==nullptr)
    {
        return false;
    }
    if(stateString == "")
    {
        return false;
    }
    std::stringstream iss( stateString );
    int dimIdx = 0;
    double val = 0;
    while ( iss >> val && dimIdx < getSpaceInformation()->getStateDimension() )
    {
        toState->as<ompl::base::RealVectorStateSpace::StateType>()->values[dimIdx] = val;
        dimIdx ++;
    }

    return true;
}

std::string MyBITstar::fromState(ompl::base::State* fromState)
{
    std::stringstream oss;
    for(unsigned int dimIdx = 0; dimIdx < getSpaceInformation()->getStateDimension(); ++dimIdx)
    {
        oss << fromState->as<ompl::base::RealVectorStateSpace::StateType>()->values[dimIdx] << " ";
    }
    return oss.str();
}

}
}
