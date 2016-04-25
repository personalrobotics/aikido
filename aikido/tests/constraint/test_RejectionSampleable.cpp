#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <aikido/constraint/RejectionSampleable.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
// #include <aikido/constraint/Sampleable.hpp>

#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>

#include "MockConstraints.hpp"
#include "../eigen_tests.hpp"

using aikido::constraint::FiniteSampleable;
using aikido::constraint::RejectionSampleable;
using aikido::constraint::TestablePtr;
using aikido::constraint::SampleablePtr;

using aikido::statespace::Rn;

class RejectionSampleableTest : public testing::Test
{
public:
  virtual void SetUp(){
    mStateSpace = std::make_shared<Rn>(1);

    std::vector<const aikido::statespace::StateSpace::State*> states;
    auto s1 = mStateSpace->createState();
    s1.setValue(aikido::tests::make_vector(1));
    auto s2 = mStateSpace->createState();
    s2.setValue(aikido::tests::make_vector(2));
    states.push_back(s1);
    states.push_back(s2);

    mPassing = std::make_shared<PassingConstraint>(mStateSpace);
    mFailing = std::make_shared<FailingConstraint>(mStateSpace);
    mSampleable = std::make_shared<FiniteSampleable>(mStateSpace, states);
  }

protected:
  TestablePtr mPassing, mFailing;
  SampleablePtr mSampleable;
  std::shared_ptr<Rn> mStateSpace;

};


TEST_F(RejectionSampleableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(RejectionSampleable(nullptr, mSampleable, mPassing, 1),
    std::invalid_argument);
}


TEST_F(RejectionSampleableTest, ConstructorThrowsOnNullSampleable)
{
  EXPECT_THROW(RejectionSampleable(mStateSpace, nullptr, mPassing, 1),
    std::invalid_argument);
}


TEST_F(RejectionSampleableTest, ConstructorThrowsOnNullTestable)
{
  EXPECT_THROW(RejectionSampleable(mStateSpace, mSampleable, nullptr, 1),
    std::invalid_argument);
}


TEST_F(RejectionSampleableTest, ConstructorThrowsOnNonMatchingStateSpaceSampleable)
{
  auto ss =  std::make_shared<Rn>(1);

  auto s1 = ss->createState();
  s1.setValue(aikido::tests::make_vector(0));

  auto sampleable = std::make_shared<FiniteSampleable>(ss, s1);
  EXPECT_THROW(RejectionSampleable(mStateSpace, sampleable, mPassing, 1),
    std::invalid_argument);
}

TEST_F(RejectionSampleableTest, ConstructorThrowsOnNonMatchingStateSpaceTestable)
{
  auto ss =  std::make_shared<Rn>(1);
  auto testable = std::make_shared<PassingConstraint>(ss);

  EXPECT_THROW(RejectionSampleable(mStateSpace, mSampleable, testable, 1),
    std::invalid_argument);
}

TEST_F(RejectionSampleableTest, ConstructorThrowsOnNegativeMaxNumTrial)
{
  EXPECT_THROW(RejectionSampleable(mStateSpace, mSampleable, mPassing, 0),
    std::invalid_argument);
  EXPECT_THROW(RejectionSampleable(mStateSpace, mSampleable, mPassing, -1),
    std::invalid_argument);
}

TEST_F(RejectionSampleableTest, GetStateSpaceMatchStateSpace)
{
  RejectionSampleable rs(mStateSpace, mSampleable, mPassing, 1);

  EXPECT_EQ(mStateSpace, rs.getStateSpace());
}

TEST_F(RejectionSampleableTest, SampleGenerator_SampleAllSamplesWithPassingTestable)
{
  RejectionSampleable rs(mStateSpace, mSampleable, mPassing, 1);

  auto rsGenerator = rs.createSampleGenerator();
  auto generator = mSampleable->createSampleGenerator();

  auto rsState = mStateSpace->createState();
  auto expected = mStateSpace->createState();
  int numSamples = generator->getNumSamples(); 
  for(int i = 0; i < numSamples; ++i)
  {
    EXPECT_TRUE(rsGenerator->sample(rsState));
    EXPECT_TRUE(generator->sample(expected));
    
    EXPECT_TRUE(mStateSpace->getValue(rsState).isApprox(
      mStateSpace->getValue(expected)));
  }

}

TEST_F(RejectionSampleableTest, SampleGenerator_RejectAllSamplesWithFailingTestable)
{
  RejectionSampleable rs(mStateSpace, mSampleable, mFailing, 1);
  auto rsGenerator = rs.createSampleGenerator();

  int numSamples = rsGenerator->getNumSamples(); 

  auto rsState = mStateSpace->createState();

  for(int i = 0; i < numSamples; ++i)
  {
    EXPECT_TRUE(rsGenerator->canSample());
    EXPECT_FALSE(rsGenerator->sample(rsState));
  }
}

