#include "constraint/CartesianProductProjectable.hpp"
#include "constraint/CartesianProductSampleable.hpp"
#include "constraint/CartesianProductTestable.hpp"
#include "constraint/CyclicSampleable.hpp"
#include "constraint/Differentiable.hpp"
#include "constraint/DifferentiableIntersection.hpp"
#include "constraint/DifferentiableSubspace.hpp"
#include "constraint/FiniteSampleable.hpp"
#include "constraint/NewtonsMethodProjectable.hpp"
#include "constraint/Projectable.hpp"
#include "constraint/RejectionSampleable.hpp"
#include "constraint/Sampleable.hpp"
#include "constraint/Satisfied.hpp"
#include "constraint/Testable.hpp"
#include "constraint/TestableIntersection.hpp"
#include "constraint/dart/CollisionFree.hpp"
#include "constraint/dart/FrameDifferentiable.hpp"
#include "constraint/dart/FramePairDifferentiable.hpp"
#include "constraint/dart/FrameTestable.hpp"
#include "constraint/dart/InverseKinematicsSampleable.hpp"
#include "constraint/dart/JointStateSpaceHelpers.hpp"
#include "constraint/dart/TSR.hpp"
#include "constraint/uniform/RnBoxConstraint.hpp"
#include "constraint/uniform/RnConstantSampler.hpp"
#include "constraint/uniform/SE2BoxConstraint.hpp"
#include "constraint/uniform/SO2UniformSampler.hpp"
#include "constraint/uniform/SO3UniformSampler.hpp"

namespace aikido {
namespace constraint {

// Add aliases for long class names here.

using R0BoxConstraint = uniform::R0BoxConstraint;
using R1BoxConstraint = uniform::R1BoxConstraint;
using R2BoxConstraint = uniform::R2BoxConstraint;
using R3BoxConstraint = uniform::R3BoxConstraint;
using R6BoxConstraint = uniform::R6BoxConstraint;
using RnBoxConstraint = uniform::RnBoxConstraint;
using R0ConstantSampler = uniform::R0ConstantSampler;
using R1ConstantSampler = uniform::R1ConstantSampler;
using R2ConstantSampler = uniform::R2ConstantSampler;
using R3ConstantSampler = uniform::R3ConstantSampler;
using R6ConstantSampler = uniform::R6ConstantSampler;
using RnConstantSampler = uniform::RnConstantSampler;
using SO2UniformSampler = uniform::SO2UniformSampler;
using SO3UniformSampler = uniform::SO3UniformSampler;
using SE2BoxConstraint = uniform::SE2BoxConstraint;

using CollisionFree = dart::CollisionFree;
using CollisionFreeOutcome = dart::CollisionFreeOutcome;
using FrameDifferentiable = dart::FrameDifferentiable;
using FramePairDifferentiable = dart::FramePairDifferentiable;
using FrameTestable = dart::FrameTestable;
using InverseKinematicsSampleable = dart::InverseKinematicsSampleable;
using TSR = dart::TSR;
using createDifferentiableBounds = dart::createDifferentiableBounds;
using createProjectableBounds = dart::createProjectableBounds;
using createTestableBounds = dart::createTestableBounds;
using createSampleableBounds = dart::createSampleableBounds;

} // namespace constraint
} // namespace aikido
