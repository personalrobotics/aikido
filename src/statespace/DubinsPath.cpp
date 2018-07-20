#include <aikido/statespace/DubinsPath.hpp>

#define DUBINS_ZERO -1e-7

namespace aikido {
namespace common {

using aikido::statespace::SE2;

void getXYYaw(const SE2::State& state, double& x, double& y, double& yaw){
    
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(state.getIsometry().rotation());

    x = state.getIsometry().translation()[0];
    y = state.getIsometry().translation()[1];
    yaw = rotation.angle();

    return;
}

SE2::State getState(const double& x, double& y, const double& yaw){
    
  Eigen::Isometry2d pose = Eigen::Isometry2d::Identity();
  pose.rotate(Eigen::Rotation2Dd(yaw));
  pose.pretranslate(Eigen::Vector2d(x, y));

  return SE2::State(pose);
}

double mod2pi(double angle){
    if (angle < 0 && angle > DUBINS_ZERO)
        return 0;
    return angle - 2.0*M_PI * floor(angle / (2.0*M_PI));
}

SE2::State interpolateLeft(const SE2::State& StateFrom,
                           double pathTravelled,
                           double turningRadius)
{
    double x, y, yaw;
    
    getXYYaw(StateFrom, x, y, yaw);

    x += (sin(yaw + pathTravelled) - sin(yaw))*turningRadius;
    y += (-cos(yaw + pathTravelled) + cos(yaw))*turningRadius;
    yaw += pathTravelled;

    return getState(x, y, mod2pi(yaw));
}

SE2::State interpolateRight(const SE2::State& StateFrom,
                            double pathTravelled,
                            double turningRadius)
{
    double x, y, yaw;
    double angle;
    getXYYaw(StateFrom, x, y, yaw);

    x += (-sin(yaw - pathTravelled) + sin(yaw))*turningRadius;
    y += ( cos(yaw - pathTravelled) - cos(yaw))*turningRadius;
    yaw -= pathTravelled;

    return getState(x, y, mod2pi(yaw));
}

SE2::State interpolateStraight(const SE2::State& StateFrom,
                               double pathTravelled,
                               double turningRadius)
{
    double x, y, yaw;
    
    getXYYaw(StateFrom, x, y, yaw);

    x += (pathTravelled*cos(yaw))*turningRadius;
    y += (pathTravelled*sin(yaw))*turningRadius;

    return getState(x, y, mod2pi(yaw));
}

class DubinsPathLSL : public AbstractDubinsPath{
public:
    DubinsPathLSL(double alpha, double d, double gamma, double turningRadius, SE2::State StateFrom)
    : mT(0)
    , mP(std::numeric_limits<double>::max())
    , mQ(0)
    , mStateFrom(StateFrom)
    {
        double ca = cos(alpha), sa = sin(alpha), cg = cos(gamma), sg = sin(gamma);
        double tmp = 2. + d * d - 2. * (ca * cg + sa * sg - d * (sa - sg));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(cg - ca, d + sa - sg);
            mT = mod2pi(-alpha + theta);
            mP = sqrt(std::max(tmp, 0.));
            mQ = mod2pi(gamma - theta);
        }
        mTurningRadius = turningRadius;
    }

    SE2::State interpolate(double alpha) override
    {
        SE2::State stateOut;
        double alphaCovered = 0;
        
        if(alpha <= mT/(mT + mP + mQ))
        {
            stateOut = interpolateLeft(mStateFrom, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(mStateFrom, mT, mTurningRadius);
            alphaCovered += mT/(mT + mP + mQ);
        }

        if(alpha <= (mT + mP)/(mT + mP + mQ))
        {
            stateOut = interpolateStraight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateStraight(stateOut, mP, mTurningRadius);
            alphaCovered += mP/(mT + mP + mQ);
        }

        if(alpha <= 1.0)
        {
            stateOut = interpolateLeft(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(stateOut, mQ, mTurningRadius);
        }
        
        stateOut = interpolateLeft(stateOut, 1.0, mTurningRadius);
        return stateOut;
    }

    double length() override
    {
        return mT + mP + mQ;
    }

private:
    double mT;
    double mP;
    double mQ;
    double mTurningRadius;
    aikido::statespace::SE2::State mStateFrom;
};

class DubinsPathRSR : public AbstractDubinsPath{
public:
    DubinsPathRSR(double alpha, double d, double gamma, double turningRadius, SE2::State StateFrom)
    : mT(0)
    , mP(std::numeric_limits<double>::max())
    , mQ(0)
    , mStateFrom(StateFrom)
    {
        double ca = cos(alpha), sa = sin(alpha), cg = cos(gamma), sg = sin(gamma);
        double tmp = 2. + d * d - 2. * (ca * cg + sa * sg - d * (sg - sa));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(ca - cg, d - sa + sg);
            mT = mod2pi(alpha - theta);
            mP = sqrt(std::max(tmp, 0.));
            mQ = mod2pi(-gamma + theta);
        }
        mTurningRadius = turningRadius;
    }

    SE2::State interpolate(double alpha) override
    {
        SE2::State stateOut;
        double alphaCovered = 0;
        
        if(alpha <= mT/(mT + mP + mQ))
        {
            stateOut = interpolateRight(mStateFrom, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(mStateFrom, mT, mTurningRadius);
            alphaCovered += mT/(mT + mP + mQ);
        }

        if(alpha <= (mT + mP)/(mT + mP + mQ))
        {
            stateOut = interpolateStraight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateStraight(stateOut, mP, mTurningRadius);
            alphaCovered += mP/(mT + mP + mQ);
        }

        if(alpha <= 1.0)
        {
            stateOut = interpolateRight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(stateOut, mQ, mTurningRadius);
        }
        
        stateOut = interpolateRight(stateOut, 1.0, mTurningRadius);
        return stateOut;
    }

    double length() override
    {
        return mT + mP + mQ;
    }

private:
    double mT;
    double mP;
    double mQ;
    double mTurningRadius;
    aikido::statespace::SE2::State mStateFrom;
};

class DubinsPathRSL : public AbstractDubinsPath{
public:
    DubinsPathRSL(double alpha, double d, double gamma, double turningRadius, SE2::State StateFrom)
    : mT(0)
    , mP(std::numeric_limits<double>::max())
    , mQ(0)
    , mStateFrom(StateFrom)
    {
        double ca = cos(alpha), sa = sin(alpha), cg = cos(gamma), sg = sin(gamma);
        double tmp = d * d - 2. + 2. * (ca * cg + sa * sg - d * (sa + sg));
        if (tmp >= DUBINS_ZERO)
        {
            mP = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cg, d - sa - sg) - atan2(2., mP);
            mT = mod2pi(alpha - theta);
            mQ = mod2pi(gamma - theta);
        }
        mTurningRadius = turningRadius;
    }
    
    SE2::State interpolate(double alpha) override
    {
        SE2::State stateOut;
        double alphaCovered = 0;
        
        if(alpha <= mT/(mT + mP + mQ))
        {
            stateOut = interpolateRight(mStateFrom, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(mStateFrom, mT, mTurningRadius);
            alphaCovered += mT/(mT + mP + mQ);
        }

        if(alpha <= (mT + mP)/(mT + mP + mQ))
        {
            stateOut = interpolateStraight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateStraight(stateOut, mP, mTurningRadius);
            alphaCovered += mP/(mT + mP + mQ);
        }

        if(alpha <= 1.0)
        {
            stateOut = interpolateLeft(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(stateOut, mQ, mTurningRadius);
        }
        
        stateOut = interpolateLeft(stateOut, 1.0, mTurningRadius);
        return stateOut;
    }

    double length() override
    {
        return mT + mP + mQ;
    }

private:
    double mT;
    double mP;
    double mQ;
    double mTurningRadius;
    aikido::statespace::SE2::State mStateFrom;
};

class DubinsPathLSR : public AbstractDubinsPath{
public:
    DubinsPathLSR(double alpha, double d, double gamma, double turningRadius, SE2::State StateFrom)
    : mT(0)
    , mP(std::numeric_limits<double>::max())
    , mQ(0)
    , mStateFrom(StateFrom)
    {
        double ca = cos(alpha), sa = sin(alpha), cg = cos(gamma), sg = sin(gamma);
        double tmp = -2. + d * d + 2. * (ca * cg + sa * sg + d * (sa + sg));
        if (tmp >= DUBINS_ZERO)
        {
            mP = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cg, d + sa + sg) - atan2(-2., mP);
            mT = mod2pi(-alpha + theta);
            mQ = mod2pi(-gamma + theta);
        }
        mTurningRadius = turningRadius;
    }

    SE2::State interpolate(double alpha) override
    {
        SE2::State stateOut;
        double alphaCovered = 0;
        
        if(alpha <= mT/(mT + mP + mQ))
        {
            stateOut = interpolateLeft(mStateFrom, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(mStateFrom, mT, mTurningRadius);
            alphaCovered += mT/(mT + mP + mQ);
        }

        if(alpha <= (mT + mP)/(mT + mP + mQ))
        {
            stateOut = interpolateStraight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateStraight(stateOut, mP, mTurningRadius);
            alphaCovered += mP/(mT + mP + mQ);
        }

        if(alpha <= 1.0)
        {
            stateOut = interpolateRight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(stateOut, mQ, mTurningRadius);
        }
        
        stateOut = interpolateRight(stateOut, 1.0, mTurningRadius);
        return stateOut;
    }

    double length() override
    {
        return mT + mP + mQ;
    }

private:
    double mT;
    double mP;
    double mQ;
    double mTurningRadius;
    aikido::statespace::SE2::State mStateFrom;
};

class DubinsPathRLR : public AbstractDubinsPath{
public:
    DubinsPathRLR(double alpha, double d, double gamma, double turningRadius, SE2::State StateFrom)
    : mT(0)
    , mP(std::numeric_limits<double>::max())
    , mQ(0)
    , mStateFrom(StateFrom)
    {
        double ca = cos(alpha), sa = sin(alpha), cg = cos(gamma), sg = sin(gamma);
        double tmp = .125 * (6. - d * d + 2. * (ca * cg + sa * sg + d * (sa - sg)));
        if (fabs(tmp) < 1.)
        {
            mP = 2.0*M_PI - acos(tmp);
            double theta = atan2(ca - cg, d - sa + sg);
            mT = mod2pi(alpha - theta + .5 * mP);
            mQ = mod2pi(alpha - gamma - mT + mP);
        }
        mTurningRadius = turningRadius;
        
    }

    SE2::State interpolate(double alpha) override
    {
        SE2::State stateOut;
        double alphaCovered = 0;
        
        if(alpha <= mT/(mT + mP + mQ))
        {
            stateOut = interpolateRight(mStateFrom, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(mStateFrom, mT, mTurningRadius);
            alphaCovered += mT/(mT + mP + mQ);
        }

        if(alpha <= (mT + mP)/(mT + mP + mQ))
        {
            stateOut = interpolateLeft(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(stateOut, mP, mTurningRadius);
            alphaCovered += mP/(mT + mP + mQ);
        }

        if(alpha <= 1.0)
        {
            stateOut = interpolateRight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(stateOut, mQ, mTurningRadius);
        }
        
        stateOut = interpolateLeft(stateOut, 1.0, mTurningRadius);
        return stateOut;
    }

    double length() override
    {
        return mT + mP + mQ;
    }

private:
    double mT;
    double mP;
    double mQ;
    double mTurningRadius;
    aikido::statespace::SE2::State mStateFrom;
};

class DubinsPathLRL : public AbstractDubinsPath{
public:
    DubinsPathLRL(double alpha, double d, double gamma, double turningRadius, SE2::State StateFrom)
    : mT(0)
    , mP(std::numeric_limits<double>::max())
    , mQ(0)
    , mStateFrom(StateFrom)
    {
        
        double ca = cos(alpha), sa = sin(alpha), cg = cos(gamma), sg = sin(gamma);
        double tmp = .125 * (6. - d * d + 2. * (ca * cg + sa * sg - d * (sa - sg)));
        if (fabs(tmp) < 1.)
        {
            mP = 2.0*M_PI - acos(tmp);
            double theta = atan2(-ca + cg, d + sa - sg);
            mT = mod2pi(-alpha + theta + .5 * mP);
            mQ = mod2pi(gamma - alpha - mT + mP);
        }
        mTurningRadius = turningRadius;
    }

    SE2::State interpolate(double alpha) override
    {
        SE2::State stateOut;
        double alphaCovered = 0;
        
        if(alpha <= mT/(mT + mP + mQ))
        {
            stateOut = interpolateLeft(mStateFrom, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(mStateFrom, mT, mTurningRadius);
            alphaCovered += mT/(mT + mP + mQ);
        }

        if(alpha <= (mT + mP)/(mT + mP + mQ))
        {
            stateOut = interpolateRight(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateRight(stateOut, mP, mTurningRadius);
            alphaCovered += mP/(mT + mP + mQ);
        }

        if(alpha <= 1.0)
        {
            stateOut = interpolateLeft(stateOut, (alpha - alphaCovered)*length(), mTurningRadius);
            return stateOut;
        }
        else
        {
            stateOut = interpolateLeft(stateOut, mQ, mTurningRadius);
        }
        
        stateOut = interpolateLeft(stateOut, 1.0, mTurningRadius);
        return stateOut;
    }

    double length() override
    {
        return mT + mP + mQ;
    }

private:
    double mT;
    double mP;
    double mQ;
    double mTurningRadius;
    aikido::statespace::SE2::State mStateFrom;
};

DubinsPath::DubinsPath(SE2::State StateFrom, SE2::State StateTo, double turningRadius)
: mStateFrom(StateFrom)
, mStateTo(StateTo)
, mTurningRadius(turningRadius)
{

    double xf, yf, yawf;
    getXYYaw(mStateFrom, xf, yf, yawf);
    
    double xt, yt, yawt;
    getXYYaw(mStateTo, xt, yt, yawt);
    
    double diffX = xt - xf;
    double diffY = yt - yf;
    
    double d = sqrt(diffX*diffX + diffY*diffY)/mTurningRadius;
    double alpha = mod2pi(yawf - atan2(diffY, diffX));
    double gamma = mod2pi(yawt - atan2(diffY, diffX));

    std::vector<AbstractDubinsPathPtr> paths;
    paths.reserve(6);
    paths.push_back(std::make_shared<DubinsPathRLR>(alpha, d, gamma, mTurningRadius, mStateFrom));
    paths.push_back(std::make_shared<DubinsPathLRL>(alpha, d, gamma, mTurningRadius, mStateFrom));
    paths.push_back(std::make_shared<DubinsPathLSR>(alpha, d, gamma, mTurningRadius, mStateFrom));
    paths.push_back(std::make_shared<DubinsPathLSL>(alpha, d, gamma, mTurningRadius, mStateFrom));
    paths.push_back(std::make_shared<DubinsPathRSL>(alpha, d, gamma, mTurningRadius, mStateFrom));
    paths.push_back(std::make_shared<DubinsPathRSR>(alpha, d, gamma, mTurningRadius, mStateFrom));

    double minLength = INFINITY;
    std::vector<AbstractDubinsPathPtr>::iterator minLengthPath;

    for(auto& path : paths)
    {   
        auto pathLength = path->length();
        if(minLength > pathLength){
            mPath = path;
            minLength = pathLength;
        }
    }

}

double DubinsPath::length()
{
   return mPath->length();
}

SE2::State DubinsPath::interpolate(double alpha)
{   
    SE2::State stateOut;

    if(alpha < 0.0)
        return mStateFrom;

    if(alpha > 1.0)
        return mStateFrom;
    
    return mPath->interpolate(alpha);
}

} //namespace common
} //namespace aikido