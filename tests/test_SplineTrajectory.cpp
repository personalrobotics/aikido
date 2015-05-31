#include <boost/assert.hpp>
#include <r3/path/SplineTrajectory.h>

using r3::path::Spline;
using r3::path::SplineTrajectory;
using Knot = Spline::Knot;


void computeDerivativeMatrix_order1()
{
  std::cout << "order1: " << std::flush;

  Eigen::MatrixXd actual = Spline::computeDerivativeMatrix(2);

  Eigen::MatrixXd expected(2, 2);
  expected << 1, 1,
              0, 1;

  BOOST_ASSERT(actual.isApprox(expected));
  std::cout << "ok" << std::endl;
}

void computeDerivativeMatrix_order2()
{
  std::cout << "order2: " << std::flush;

  Eigen::MatrixXd actual = Spline::computeDerivativeMatrix(3);

  Eigen::MatrixXd expected(3, 3);
  expected << 1, 1, 1,
              0, 1, 2,
              0, 0, 2;

  BOOST_ASSERT(actual.isApprox(expected));
  std::cout << "ok" << std::endl;
}

void computeDerivativeMatrix_order3()
{
  std::cout << "order3: " << std::flush;

  Eigen::MatrixXd actual = Spline::computeDerivativeMatrix(4);

  Eigen::MatrixXd expected(4, 4);
  expected << 1, 1, 1, 1,
              0, 1, 2, 3,
              0, 0, 2, 6,
              0, 0, 0, 6;

  BOOST_ASSERT(actual.isApprox(expected));
  std::cout << "ok" << std::endl;
}

void createProblem_order1_waypoints2()
{
  std::vector<Knot> knots(2);
  knots[0].t = 0.;
  knots[0].values.resize(1, 1);
  knots[0].values << -1;
  knots[1].t = 1.;
  knots[1].values.resize(1, 1);
  knots[1].values << 1;

  Spline::Problem actual = Spline::createProblem(knots);

  std::cout << actual.A << std::endl;
}

void createProblem_order1_waypoints3()
{
  std::vector<Knot> knots(3);
  knots[0].t = 0.;
  knots[0].values.resize(1, 1);
  knots[0].values << -1;
  knots[1].t = 1.;
  knots[1].values.resize(1, 1);
  knots[1].values << 1;
  knots[2].t = 2.;
  knots[2].values.resize(1, 1);
  knots[2].values << 3;

  Spline::Problem actual = Spline::createProblem(knots);

  std::cout << actual.A << std::endl;
}

void createProblem_order3_waypoints3()
{
  std::vector<Knot> knots(3);
  knots[0].t = 0.;
  knots[0].values.resize(2, 1);
  knots[0].values << -1, 0;
  knots[1].t = 1.;
  knots[1].values.resize(2, 1);
  knots[1].values << 1, -1;
  knots[2].t = 2.;
  knots[2].values.resize(2, 1);
  knots[2].values << 3, 0;

  std::vector<Spline> splines = Spline::fit(knots);

  for (double t = 0; t <= 2 + 1e-3; t += 0.01) {
    std::cout << t << "\t" << splines[0].interpolate(t) << std::endl;
  }
}

int main(int argc, char **argv)
{
  computeDerivativeMatrix_order1();
  computeDerivativeMatrix_order2();
  computeDerivativeMatrix_order3();
  createProblem_order1_waypoints2();
  createProblem_order1_waypoints3();
  createProblem_order3_waypoints3();

  return 0;
}
