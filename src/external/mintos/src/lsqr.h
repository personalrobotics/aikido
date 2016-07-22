/*
* lsqr.h
* Contains auxiliary functions, data type definitions, and function 
* prototypes for the iterative linear solver LSQR. 
*
* 08 Sep 1999: First version from James W. Howse <jhowse@lanl.gov>
*/

/*
*------------------------------------------------------------------------------
*
*     LSQR  finds a solution x to the following problems:
*
*     1. Unsymmetric equations --    solve  A*x = b
*
*     2. Linear least squares  --    solve  A*x = b
*                                    in the least-squares sense
*
*     3. Damped least squares  --    solve  (   A    )*x = ( b )
*                                           ( damp*I )     ( 0 )
*                                    in the least-squares sense
*
*     where 'A' is a matrix with 'm' rows and 'n' columns, 'b' is an
*     'm'-vector, and 'damp' is a scalar.  (All quantities are real.)
*     The matrix 'A' is intended to be large and sparse.  
*
*
*     Notation
*     --------
*
*     The following quantities are used in discussing the subroutine
*     parameters:
*
*     'Abar'   =  (   A    ),          'bbar'  =  ( b )
*                 ( damp*I )                      ( 0 )
*
*     'r'      =  b  -  A*x,           'rbar'  =  bbar  -  Abar*x
*
*     'rnorm'  =  sqrt( norm(r)**2  +  damp**2 * norm(x)**2 )
*              =  norm( rbar )
*
*     'rel_prec'  =  the relative precision of floating-point arithmetic
*                    on the machine being used.  For example, on the IBM 370,
*                    'rel_prec' is about 1.0E-6 and 1.0D-16 in single and double
*                    precision respectively.
*
*     LSQR  minimizes the function 'rnorm' with respect to 'x'.
*
*------------------------------------------------------------------------------
*/

#include "stdio.h"
#include <mintos/math/VectorTemplate.h>
using namespace Math;

/*
*------------------------------------------------------------------------------
*
*     Input Quantities
*     ----------------
*
*     num_rows     input  The number of rows (e.g., 'm') in the matrix A.
*
*     num_cols     input  The number of columns (e.g., 'n') in the matrix A.
*
*     damp_val     input  The damping parameter for problem 3 above.
*                         ('damp_val' should be 0 for problems 1 and 2.)
*                         If the system A*x = b is incompatible, values
*                         of 'damp_val' in the range
*                            0 to sqrt('rel_prec')*norm(A)
*                         will probably have a negligible effect.
*                         Larger values of 'damp_val' will tend to decrease
*                         the norm of x and reduce the number of
*                         iterations required by LSQR.
*
*                         The work per iteration and the storage needed
*                         by LSQR are the same for all values of 'damp_val'.
*
*     rel_mat_err  input  An estimate of the relative error in the data
*                         defining the matrix 'A'.  For example,
*                         if 'A' is accurate to about 6 digits, set
*                         rel_mat_err = 1.0e-6 .
*
*     rel_rhs_err  input  An extimate of the relative error in the data
*                         defining the right hand side (rhs) vector 'b'.  For 
*                         example, if 'b' is accurate to about 6 digits, set
*                         rel_rhs_err = 1.0e-6 .
*
*     cond_lim     input  An upper limit on cond('Abar'), the apparent
*                         condition number of the matrix 'Abar'.
*                         Iterations will be terminated if a computed
*                         estimate of cond('Abar') exceeds 'cond_lim'.
*                         This is intended to prevent certain small or
*                         zero singular values of 'A' or 'Abar' from
*                         coming into effect and causing unwanted growth
*                         in the computed solution.
*
*                         'cond_lim' and 'damp_val' may be used separately or
*                         together to regularize ill-conditioned systems.
*
*                         Normally, 'cond_lim' should be in the range
*                         1000 to 1/rel_prec.
*                         Suggested value:
*                         cond_lim = 1/(100*rel_prec)  for compatible systems,
*                         cond_lim = 1/(10*sqrt(rel_prec)) for least squares.
*
*             Note:  If the user is not concerned about the parameters
*             'rel_mat_err', 'rel_rhs_err' and 'cond_lim', any or all of them 
*             may be set to zero.  The effect will be the same as the values
*             'rel_prec', 'rel_prec' and 1/rel_prec respectively.
*
*     max_iter     input  An upper limit on the number of iterations.
*                         Suggested value:
*                         max_iter = n/2   for well-conditioned systems
*                                          with clustered singular values,
*                         max_iter = 4*n   otherwise.
*
*     lsqr_fp_out  input  Pointer to the file for sending printed output.  If  
*                         not NULL, a summary will be printed to the file that 
*                         'lsqr_fp_out' points to.
*
*     rhs          input  The right hand side (rhs) vector 'b'.  This vector
*                         has a length of 'm' (i.e., 'num_rows').  The routine 
*                         LSQR is designed to over-write 'rhs'.
*
*     sol          input  The initial guess for the solution vector 'x'.  This 
*                         vector has a length of 'n' (i.e., 'num_cols').  The  
*                         routine LSQR is designed to over-write 'sol'.
*
*------------------------------------------------------------------------------
*/

struct lsqr_input
{
  long     num_rows,num_cols;
  double   damp_val;
  double   rel_mat_err,rel_rhs_err;
  double   cond_lim;
  long     max_iter;
  FILE     *lsqr_fp_out;
  dVector  rhs;
  dVector  sol;
};

/*
*------------------------------------------------------------------------------
*
*     Output Quantities
*     -----------------
*
*     term_flag       output  An integer giving the reason for termination:
*
*                     0       x = x0  is the exact solution.
*                             No iterations were performed.
*
*                     1       The equations A*x = b are probably compatible.  
*                             Norm(A*x - b) is sufficiently small, given the 
*                             values of 'rel_mat_err' and 'rel_rhs_err'.
*
*                     2       The system A*x = b is probably not
*                             compatible.  A least-squares solution has
*                             been obtained that is sufficiently accurate,
*                             given the value of 'rel_mat_err'.
*
*                     3       An estimate of cond('Abar') has exceeded
*                             'cond_lim'.  The system A*x = b appears to be
*                             ill-conditioned.  Otherwise, there could be an
*                             error in subroutine APROD.
*
*                     4       The equations A*x = b are probably
*                             compatible.  Norm(A*x - b) is as small as
*                             seems reasonable on this machine.
*
*                     5       The system A*x = b is probably not
*                             compatible.  A least-squares solution has
*                             been obtained that is as accurate as seems
*                             reasonable on this machine.
*
*                     6       Cond('Abar') seems to be so large that there is
*                             no point in doing further iterations,
*                             given the precision of this machine.
*                             There could be an error in subroutine APROD.
*
*                     7       The iteration limit 'max_iter' was reached.
*  
*     num_iters       output  The number of iterations performed.
*
*     frob_mat_norm   output  An estimate of the Frobenius norm of 'Abar'.
*                             This is the square-root of the sum of squares
*                             of the elements of 'Abar'.
*                             If 'damp_val' is small and if the columns of 'A'
*                             have all been scaled to have length 1.0,
*                             'frob_mat_norm' should increase to roughly
*                             sqrt('n').
*                             A radically different value for 'frob_mat_norm' 
*                             may indicate an error in subroutine APROD (there
*                             may be an inconsistency between modes 1 and 2).
*
*     mat_cond_num    output  An estimate of cond('Abar'), the condition
*                             number of 'Abar'.  A very high value of 
*                             'mat_cond_num'
*                             may again indicate an error in APROD.
*
*     resid_norm      output  An estimate of the final value of norm('rbar'),
*                             the function being minimized (see notation
*                             above).  This will be small if A*x = b has
*                             a solution.
*
*     mat_resid_norm  output  An estimate of the final value of
*                             norm( Abar(transpose)*rbar ), the norm of
*                             the residual for the usual normal equations.
*                             This should be small in all cases.  
*                             ('mat_resid_norm'
*                             will often be smaller than the true value
*                             computed from the output vector 'x'.)
*
*     sol_norm        output  An estimate of the norm of the final
*                             solution vector 'x'.
*
*     sol             output  The vector which returns the computed solution 
*                             'x'.
*                             This vector has a length of 'n' (i.e., 
*                             'num_cols').
*
*     std_err         output  The vector which returns the standard error 
*                             estimates  for the components of 'x'.
*                             This vector has a length of 'n'
*                             (i.e., 'num_cols').  For each i, std_err(i) 
*                             is set to the value
*                             'resid_norm' * sqrt( sigma(i,i) / T ),
*                             where sigma(i,i) is an estimate of the i-th
*                             diagonal of the inverse of Abar(transpose)*Abar
*                             and  T = 1      if  m <= n,
*                                  T = m - n  if  m > n  and  damp_val = 0,
*                                  T = m      if  damp_val != 0.
*
*------------------------------------------------------------------------------
*/

struct lsqr_output
{
  enum { X0Exact=0, ExactSolutionRelMat=1, LSSolutionRelMat=2,
	 IllConditioned=3, 
	 ExactSolution=4, LSSolution=5,
	 ConditionError=6, MaxItersReached=7 };
  long     term_flag;
  long     num_iters;
  double   frob_mat_norm;
  double   mat_cond_num;
  double   resid_norm;
  double   mat_resid_norm;
  double   sol_norm;
  dVector  sol;
  dVector  std_err;
};

/*
*------------------------------------------------------------------------------
*
*     Workspace Quantities
*     --------------------
*
*     bidiag_wrk    workspace  This float vector is a workspace for the 
*                              current iteration of the
*                              Lanczos bidiagonalization.  
*                              This vector has length 'n' (i.e., 'num_cols').
*
*     srch_dir      workspace  This float vector contains the search direction 
*                              at the current iteration.  This vector has a 
*                              length of 'n' (i.e., 'num_cols').
*
*------------------------------------------------------------------------------
*/

struct lsqr_work 
{
  dVector  bidiag_wrk;
  dVector  srch_dir;
};

/** A function that performs matrix multiplications */
class lsqr_func
{
public:
  /* compute  y = y + A*x*/
  virtual void MatrixVectorProduct (const dVector& x, dVector& y) =0; 
  /* compute  x = x + At*y*/
  virtual void MatrixTransposeVectorProduct (dVector& x, const dVector& y) =0;
};

void lsqr( lsqr_input&, lsqr_output&, lsqr_work&, lsqr_func&);

