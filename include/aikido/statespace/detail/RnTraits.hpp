namespace aikido {
namespace statespace {
namespace detail {

//==============================================================================
template <int N>
struct RnTraits
{
  using DartJoint
      = ::dart::dynamics::GenericJoint<::dart::math::RealVectorSpace<N>>;
};

//==============================================================================
template <>
struct RnTraits<0>
{
  using DartJoint = ::dart::dynamics::WeldJoint;
};

} // namespace detail
} // namespace statespace
} // namespace aikido
