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

using uniform::R0BoxConstraint;
using uniform::R1BoxConstraint;
using uniform::R2BoxConstraint;
using uniform::R3BoxConstraint;
using uniform::R6BoxConstraint;
using uniform::RnBoxConstraint;
using uniform::R0ConstantSampler;
using uniform::R1ConstantSampler;
using uniform::R2ConstantSampler;
using uniform::R3ConstantSampler;
using uniform::R6ConstantSampler;
using uniform::RnConstantSampler;
using uniform::SO2UniformSampler;
using uniform::SO3UniformSampler;
using uniform::SE2BoxConstraint;

using dart::CollisionFree;
using dart::CollisionFreeOutcome;
using dart::FrameDifferentiable;
using dart::FramePairDifferentiable;
using dart::FrameTestable;
using dart::InverseKinematicsSampleable;
using dart::TSR;
using dart::createDifferentiableBounds;
using dart::createProjectableBounds;
using dart::createTestableBounds;
using dart::createSampleableBounds;

} // namespace constraint
} // namespace aikido
