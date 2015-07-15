#include <limits>
#include <vector>
#include <Eigen/Core>

/*
 * Knot
 */
template <int _Derivatives = Eigen::Dynamic,
          int _Values = Eigen::Dynamic>
class Knot {
public:
  using Value = Eigen::Matrix<double, _Values, 1>;

  enum ConstraintType {
    CONSTRAINT_NONE,
    CONSTRAINT_VALUE,
    CONSTRAINT_EQUAL
  };

  Knot();
  Knot(size_t _num_derivatives, size_t _num_values);

  int getNumValues() const;
  int getNumDerivatives() const;

  void setTime(double _time);
  void setValue(int _derivative, const Value& _value);
  void setEqual(int _derivative, int _target);
  void removeConstraint(int _derivative);

private:
  double mTime;
  std::vector<ConstraintType> mConstraintTypes;
  std::vector<int> mTargets;
  Eigen::Matrix<double, _Derivatives, _Values> mValues;
};

// ---

template <int _Derivatives, int _Values>
void Knot<_Derivatives>::Knot()
  : Knot(_Derivatives, _Values)
{
}

template <int _Derivatives, int _Values>
int Knot<_Derivatives>::getNumValues() const
{
  return mValues.cols();
}

template <int _Derivatives, int _Values>
int Knot<_Derivatives>::getNumDerivatives() const
{
  return mValues.rows();
}

template <int _Derivatives, int _Values>
void Knot<_Derivatives, int _Values>::Knot(size_t _num_derivatives, size_t _num_values)
  : mTime(0.),
    mConstraintTypes(_num_derivatives, CONSTRAINT_NONE),
    mTargets(_num_derivatives),
    mValues(_num_derivatives)
{
}

template <int _Derivatives, int _Values>
void Knot<_Derivatives, int _Values>::setTime(double _time)
{
  mTime = _time;
}

template <int _Derivatives, int _Values>
void Knot<_Derivatives, int _Values>::setValue(int _derivative, const Value& _value)
{
  mConstraintTypes[_derivative] = CONSTRAINT_VALUE;
  mValues.row(_derivative) = _value;
}

template <int _Derivatives, int _Values>
void Knot<_Derivatives, int _Values>::setValue(int _derivative, int _target)
{
  mConstraintTypes[_derivative] = CONSTRAINT_EQUAL;
  mTargets[_derivative] = _target;
}

template <int _Derivatives, int _Values>
void Knot<_Derivatives, int _Values>::removeConstraint(int _derivative)
{
  mConstraintType[_derivative] = CONSTRAINT_NONE;
}

/*
 * Problem
 */
template <int _Derivatives = Eigen::Dynamic,
          int _Values = Eigen::Dynamic>
class Problem {
public:
  using KnotType = Knot<_Derivatives, _Values>;

  template <class Iterator>
  Problem(Iterator _begin, Iterator _end);

  void solve();

private:
  std::vector<KnotType> mKnots;
  Eigen::MatrixXd mA;
  Eigen::Matrix<double, Eigen::Dynamic, _Values> mB;
};

// ---

template <int _Derivatives, int _Values, class Iterator>
Problem<_Derivatives>::Problem(Iterator _begin, Iterator _end)
  : mKnots(_begin, _end)
{
}
template <int _Derivatives, int _Values>
void Problem<_Derivatives>::solve()
{
  const size_t numValues = mKnots.

}



