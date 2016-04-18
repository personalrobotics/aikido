#include <aikido/util/StepSequence.hpp>

StepSequence::StepSequence(const double _stepSize, const bool _includeEndpoints)
    : mStepSize(_stepSize)
    , mIncludeEndpoints(_includeEndpoints)
{
}

pair<double, double> StepSequence::operator[](int n)
{
    pair<double, double> val;

    
