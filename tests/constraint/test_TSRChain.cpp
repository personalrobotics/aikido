#include <aikido/constraint/TSRChain.hpp>

using aikido::constraint::TSRChain;

TEST(TSRChain, Constructor)
{

}

TEST(TSRChain, CopyConstructor)
{
  TSRChain tsr_chain;
  TSRChain tsr_chain2( tsr_chain );

  EXPECT_EQ(tsr_chain.mTSRs.size(), tsr_chain2.mTSRs.size());
  for(unsigned int i=0; i < tsr_chain.mTSRs.size(); i++)
  {
    EXPECT_EQ(tsr_chain.mTSRs[i].mT0_w, 
              tsr_chain2.mTSRs[i].mT0_w);
    EXPECT_EQ(tsr_chain.mTSRs[i].mBw, 
              tsr_chain2.mTSRs[i].mBw);
    EXPECT_EQ(tsr_chain.mTSRs[i].mTw_e, 
              tsr_chain2.mTSRs[i].mTw_e);
  }
}

TEST(TSRChain, AssignmentOperator)
{

}

TEST(TSRChain, Validate)
{

}

TEST(TSRChainSampleGenerator, SamplePointTSR)
{


}
