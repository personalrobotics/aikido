#include "aikido/constraint/CartesianProductProjectable.hpp"
#include "aikido/constraint/CartesianProductSampleable.hpp"
#include "aikido/constraint/CartesianProductTestable.hpp"
#include "aikido/constraint/CyclicSampleable.hpp"
#include "aikido/constraint/Differentiable.hpp"
#include "aikido/constraint/DifferentiableIntersection.hpp"
#include "aikido/constraint/DifferentiableSubspace.hpp"
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/NewtonsMethodProjectable.hpp"
#include "aikido/constraint/Projectable.hpp"
#include "aikido/constraint/RejectionSampleable.hpp"
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Satisfied.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/FrameDifferentiable.hpp"
#include "aikido/constraint/dart/FramePairDifferentiable.hpp"
#include "aikido/constraint/dart/FrameTestable.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/constraint/uniform/RnBoxConstraint.hpp"
#include "aikido/constraint/uniform/RnConstantSampler.hpp"
#include "aikido/constraint/uniform/SE2BoxConstraint.hpp"
#include "aikido/constraint/uniform/SO2UniformSampler.hpp"
#include "aikido/constraint/uniform/SO3UniformSampler.hpp"

namespace aikido {
namespace constraint {

// Add aliases for long class names here.

using uniform::R0BoxConstraint;
using uniform::R0ConstantSampler;
using uniform::R1BoxConstraint;
using uniform::R1ConstantSampler;
using uniform::R2BoxConstraint;
using uniform::R2ConstantSampler;
using uniform::R3BoxConstraint;
using uniform::R3ConstantSampler;
using uniform::R6BoxConstraint;
using uniform::R6ConstantSampler;
using uniform::RnBoxConstraint;
using uniform::RnConstantSampler;
using uniform::SE2BoxConstraint;
using uniform::SO2UniformSampler;
using uniform::SO3UniformSampler;

using dart::CollisionFree;
using dart::CollisionFreeOutcome;
using dart::createDifferentiableBounds;
using dart::createProjectableBounds;
using dart::createSampleableBounds;
using dart::createTestableBounds;
using dart::FrameDifferentiable;
using dart::FramePairDifferentiable;
using dart::FrameTestable;
using dart::InverseKinematicsSampleable;
using dart::TSR;

} // namespace constraint
} // namespace aikido
