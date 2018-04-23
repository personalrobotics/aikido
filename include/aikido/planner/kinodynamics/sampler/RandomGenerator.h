#ifndef RANDOM_GNR_H_
#define RANDOM_GNR_H_

#include <random>

template <typename T>
class RandomGenerator
{
public:
    RandomGenerator(std::mt19937::result_type seed = std::random_device{}())
        : gen_(seed)
    {
    }

    void setSeed(std::mt19937::result_type seed)
    {
        gen_.seed(seed);
    }

    virtual T sample() = 0;
protected:
    std::mt19937 gen_;
};

class UniformRealRandomGenerator : public RandomGenerator<double>
{
public:
    UniformRealRandomGenerator(std::mt19937::result_type seed = std::random_device{}())
        : RandomGenerator(seed)
    {
    }

    virtual double sample() override
    {
        return sample(0.0, 1.0);
    }

    double sample(double minVal, double maxVal)
    {
        return std::uniform_real_distribution<double>(minVal, maxVal)(gen_);
    }

    std::vector<double> sample(double minVal, double maxVal, int num)
    {
        std::vector<double> vec(0.0, num);
        for (int i = 0; i < num; i++)
        {
            vec[i] = sample(minVal, maxVal);
        }
        return vec;
    }
};

class NormalRealRandomGenerator : public RandomGenerator<double>
{
public:
    NormalRealRandomGenerator(double meanVal = 0.0, double varianceVal = 1.0,
                              std::mt19937::result_type seed = std::random_device{}())
        :  RandomGenerator(seed)
    {
        mean_ = meanVal;
        variance_ = varianceVal;
    }

    virtual double sample() override
    {
        return sample(mean_, variance_);
    }

    double sample(double meanVal, double varianceVal)
    {
        return std::normal_distribution<double>(meanVal, varianceVal)(gen_);
    }

protected:
    double mean_;
    double variance_;
};


#endif // RANDOM_GNR_H_
