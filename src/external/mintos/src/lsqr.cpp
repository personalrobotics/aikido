/*
* lsqr.c
* This is a C version of LSQR, derived from the Fortran 77 implementation
* of C. C. Paige and M. A. Saunders.
*
* This file defines functions for data type allocation and deallocation,
* lsqr itself (the main algorithm),
* and functions that scale, copy, and compute the Euclidean norm of a vector.
*
* 08 Sep 1999: First version from James W. Howse <jhowse@lanl.gov>
*/

#include "lsqr.h"

using namespace std;

static char* lsqr_term_msg[8] = {
  "The exact solution is x = x0",
  "The residual Ax - b is small enough, given ATOL and BTOL",
  "The least squares error is small enough, given ATOL",
  "The estimated condition number has exceeded CONLIM",
  "The residual Ax - b is small enough, given machine precision",
  "The least squares error is small enough, given machine precision",
  "The estimated condition number has exceeded machine precision",
  "The iteration limit has been reached"
};

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
*                    on the machine being used.  Typically 2.22e-16
*                    with 64-bit arithmetic.
*
*     LSQR  minimizes the function 'rnorm' with respect to 'x'.
*
*
*     References
*     ----------
*
*     C.C. Paige and M.A. Saunders,  LSQR: An algorithm for sparse
*          linear equations and sparse least squares,
*          ACM Transactions on Mathematical Software 8, 1 (March 1982),
*          pp. 43-71.
*
*     C.C. Paige and M.A. Saunders,  Algorithm 583, LSQR: Sparse
*          linear equations and least-squares problems,
*          ACM Transactions on Mathematical Software 8, 2 (June 1982),
*          pp. 195-209.
*
*     C.L. Lawson, R.J. Hanson, D.R. Kincaid and F.T. Krogh,
*          Basic linear algebra subprograms for Fortran usage,
*          ACM Transactions on Mathematical Software 5, 3 (Sept 1979),
*          pp. 308-323 and 324-325.
*
*------------------------------------------------------------------------------
*/
void lsqr( lsqr_input &input, lsqr_output &output, lsqr_work &work,
           lsqr_func &func)
{
  /*
   *     The convergence criteria are required to be met on NCONV consecutive 
   *     iterations, where NCONV is set below.  Suggested values are 1, 2, or 3.
   */
  long    term_iter = 0,
    term_iter_max = 1;

  double  alpha=0, 
    beta,
    bnorm,
    bbnorm=0,
    ddnorm=0,
    xxnorm=0;
  double  cs2=-1,
    sn2=0,
    zeta=0,
    res=0,
    cond_tol;

  if( input.lsqr_fp_out != NULL )
    fprintf( input.lsqr_fp_out, "  Least Squares Solution of A*x = b\n\
	The matrix A has %7i rows and %7i columns\n\
	The damping parameter is\tDAMP = %10.2e\n\
	ATOL = %10.2e\t\tCONDLIM = %10.2e\n\
	BTOL = %10.2e\t\tITERLIM = %10i\n\n",
	     input.num_rows, input.num_cols, input.damp_val, input.rel_mat_err,
	     input.cond_lim, input.rel_rhs_err, input.max_iter );
  
  output.term_flag = 0;
  output.num_iters = 0;

  output.frob_mat_norm = 0.0;
  output.mat_cond_num = 0.0;
  output.sol_norm = 0.0;

  work.bidiag_wrk.resize(input.num_cols,0);  
  work.srch_dir.resize(input.num_cols,0);
  output.std_err.resize(input.num_cols,0);

  if( input.cond_lim > 0.0 )
    cond_tol = 1.0 / input.cond_lim;
  else
    cond_tol = DBL_EPSILON;

  /*
   *  Set up the initial vectors u and v for bidiagonalization.  These satisfy 
   *  the relations
   *             BETA*u = b - A*x0 
   *             ALPHA*v = A^T*u
   */

  /* Compute b - A*x0 and store in vector u which initially held vector b */
  input.rhs.inplaceNegative();
  func.MatrixVectorProduct(input.sol, input.rhs );
  input.rhs.inplaceNegative();
  output.sol = input.sol;

  /* compute Euclidean length of u and store as BETA */
  beta = input.rhs.norm();
  
  if( beta > 0.0 ) {
    /* scale vector u by the inverse of BETA */
    input.rhs.inplaceMul(1.0 / beta);
    
    /* Compute matrix-vector product A^T*u and store it in vector v */
    func.MatrixTransposeVectorProduct(work.bidiag_wrk, input.rhs);
    
    /* compute Euclidean length of v and store as ALPHA */
    alpha = work.bidiag_wrk.norm(); 
  }
  
  if( alpha > 0.0 )  {
    /* scale vector v by the inverse of ALPHA */
    work.bidiag_wrk.inplaceMul(1.0 / alpha);

    /* copy vector v to vector w */
    work.srch_dir.copy(work.bidiag_wrk);
  }    

  output.mat_resid_norm = alpha * beta;
  output.resid_norm = beta;
  bnorm = beta;
  /*
   *  If the norm || A^T r || is zero, then the initial guess is the exact
   *  solution.  Exit and report this.
   */
  if( (output.mat_resid_norm == 0.0) && (input.lsqr_fp_out != NULL) ) {
    fprintf( input.lsqr_fp_out, "\tISTOP = %3i\t\t\tITER = %9i\n\
	|| A ||_F = %13.5e\tcond( A ) = %13.5e\n\
	|| r ||_2 = %13.5e\t|| A^T r ||_2 = %13.5e\n\
	|| b ||_2 = %13.5e\t|| x - x0 ||_2 = %13.5e\n\n", 
	     output.term_flag, output.num_iters, output.frob_mat_norm, 
	     output.mat_cond_num, output.resid_norm, output.mat_resid_norm,
	     bnorm, output.sol_norm );
      
    fprintf( input.lsqr_fp_out, "  %s\n\n", lsqr_term_msg[output.term_flag]);
      
    return;
  }

  double rhobar = alpha;
  double phibar = beta;
  /*
   *  If statistics are printed at each iteration, print a header and the initial
   *  values for each quantity.
   */
  if( input.lsqr_fp_out != NULL )  {
    fprintf( input.lsqr_fp_out,
	     "  ITER     || r ||    Compatible  ||A^T r|| / ||A|| ||r||  || A ||    cond( A )\n\n" );

    double stop_crit_1 = 1.0;
    double stop_crit_2 = alpha / beta;

    fprintf( input.lsqr_fp_out,
	     "%6i %13.5e %10.2e \t%10.2e \t%10.2e  %10.2e\n",
	     output.num_iters, output.resid_norm, stop_crit_1, stop_crit_2,
	     output.frob_mat_norm, output.mat_cond_num);
  }
 
  /*
   *  The main iteration loop is continued as long as no stopping criteria
   *  are satisfied and the number of total iterations is less than some upper
   *  bound.
   */
  while( output.term_flag == 0 )  {
    output.num_iters++;
    /*      
     *     Perform the next step of the bidiagonalization to obtain
     *     the next vectors u and v, and the scalars ALPHA and BETA.
     *     These satisfy the relations
     *                BETA*u  =  A*v  -  ALPHA*u,
     *                ALFA*v  =  A^T*u  -  BETA*v.
     */      
    /* scale vector u by the negative of ALPHA */
    input.rhs.inplaceMul(-alpha);

    /* compute A*v - ALPHA*u and store in vector u */
    func.MatrixVectorProduct(work.bidiag_wrk, input.rhs );

    /* compute Euclidean length of u and store as BETA */
    beta = input.rhs.norm();

    /* accumulate this quantity to estimate Frobenius norm of matrix A */
    bbnorm += Sqr(alpha) + Sqr(beta) + Sqr(input.damp_val);

    if( beta > 0.0 )	{
      /* scale vector u by the inverse of BETA */
      input.rhs.inplaceMul(1.0 / beta);

      /* scale vector v by the negative of BETA */
      work.bidiag_wrk.inplaceMul(-beta);

      /* compute A^T*u - BETA*v and store in vector v */
      func.MatrixTransposeVectorProduct( work.bidiag_wrk, input.rhs );
      
      /* compute Euclidean length of v and store as ALPHA */
      alpha = work.bidiag_wrk.norm();

      if( alpha > 0.0 )
	/* scale vector v by the inverse of ALPHA */
	work.bidiag_wrk.inplaceMul(1.0 / alpha);
    }
    /*
     *     Use a plane rotation to eliminate the damping parameter.
     *     This alters the diagonal (RHOBAR) of the lower-bidiagonal matrix.
     */
    double cs1 = rhobar / Sqrt( Sqr(rhobar) + Sqr(input.damp_val) );
    double sn1 = input.damp_val / Sqrt( Sqr(rhobar) + Sqr(input.damp_val) );
      
    double psi = sn1 * phibar;
    phibar = cs1 * phibar;
    /*      
     *     Use a plane rotation to eliminate the subdiagonal element (BETA)
     *     of the lower-bidiagonal matrix, giving an upper-bidiagonal matrix.
     */
    double rho = Sqrt( Sqr(rhobar) + Sqr(input.damp_val) + Sqr(beta) );
    double cs = Sqrt( Sqr(rhobar) + Sqr(input.damp_val) ) / rho;
    double sn = beta / rho;

    double theta = sn * alpha;
    rhobar = -cs * alpha;
    double phi = cs * phibar;
    phibar = sn * phibar;
    double tau = sn * phi;
    /*
     *     Update the solution vector x, the search direction vector w, and the 
     *     standard error estimates vector se.
     */     
    /* update the solution vector x */
    output.sol.madd(work.srch_dir, phi/rho);
    for(int i = 0; i < input.num_cols; i++) {
      /* update the standard error estimates vector se */
      output.std_err(i) += Sqr( (1.0 / rho) * work.srch_dir(i) );
    }

    /* accumulate this quantity to estimate condition number of A */
    ddnorm += Sqr( (1.0 / rho))*work.srch_dir.normSquared();

    /* update the search direction vector w */
    work.srch_dir.inplaceMul(-theta/rho);
    work.srch_dir += work.bidiag_wrk;
    /*
     *     Use a plane rotation on the right to eliminate the super-diagonal element
     *     (THETA) of the upper-bidiagonal matrix.  Then use the result to estimate 
     *     the solution norm || x ||.
     */
    double delta = sn2 * rho;
    double gammabar = -cs2 * rho;
    double zetabar = (phi - delta * zeta) / gammabar;

    /* compute an estimate of the solution norm || x || */
    output.sol_norm = Sqrt( xxnorm + Sqr(zetabar) );

    double gamma = Sqrt( Sqr(gammabar) + Sqr(theta) );
    cs2 = gammabar / gamma;
    sn2 = theta / gamma;
    zeta = (phi - delta * zeta) / gamma;

    /* accumulate this quantity to estimate solution norm || x || */
    xxnorm += Sqr(zeta);
    /*
     *     Estimate the Frobenius norm and condition of the matrix A, and the 
     *     Euclidean norms of the vectors r and A^T*r.
     */
    output.frob_mat_norm = Sqrt( bbnorm );
    output.mat_cond_num = output.frob_mat_norm * Sqrt( ddnorm );

    res += Sqr(psi);
    output.resid_norm = Sqrt( Sqr(phibar) + res );


    output.mat_resid_norm = alpha * fabs( tau );
    /*
     *     Use these norms to estimate the values of the three stopping criteria.
     */
    double stop_crit_1 = output.resid_norm / bnorm;

    double stop_crit_2 = 0.0;
    if( output.resid_norm > 0.0 )
      stop_crit_2 = output.mat_resid_norm / ( output.frob_mat_norm * 
					      output.resid_norm );

    double stop_crit_3 = 1.0 / output.mat_cond_num;

    double resid_tol = input.rel_rhs_err + input.rel_mat_err * 
      output.mat_resid_norm * output.sol_norm / bnorm;

    double resid_tol_mach = DBL_EPSILON + DBL_EPSILON * output.mat_resid_norm * output.sol_norm / bnorm;
    /*
     *     Check to see if any of the stopping criteria are satisfied.
     *     First compare the computed criteria to the machine precision.
     *     Second compare the computed criteria to the the user specified precision.
     */
    /* iteration limit reached */
    if( output.num_iters >= input.max_iter )
      output.term_flag = 7;

    /* condition number greater than machine precision */
    if( stop_crit_3 <= DBL_EPSILON )
      output.term_flag = 6;
    /* least squares error less than machine precision */
    if( stop_crit_2 <= DBL_EPSILON )
      output.term_flag = 5;
    /* residual less than a function of machine precision */
    if( stop_crit_1 <= resid_tol_mach )
      output.term_flag = 4;

    /* condition number greater than CONLIM */
    if( stop_crit_3 <= cond_tol )
      output.term_flag = 3;
    /* least squares error less than ATOL */
    if( stop_crit_2 <= input.rel_mat_err )
      output.term_flag = 2;
    /* residual less than a function of ATOL and BTOL */
    if( stop_crit_1 <= resid_tol )
      output.term_flag = 1;
    /*
     *  If statistics are printed at each iteration, print a header and the initial
     *  values for each quantity.
     */
    if( input.lsqr_fp_out != NULL )
      {
	fprintf( input.lsqr_fp_out,
		 "%6i %13.5e %10.2e \t%10.2e \t%10.2e %10.2e\n",
		 output.num_iters, output.resid_norm, stop_crit_1, 
		 stop_crit_2,
		 output.frob_mat_norm, output.mat_cond_num);
      }
    if( output.term_flag == 0 )
      term_iter = -1;

    term_iter++;

    if( (term_iter < term_iter_max) &&
	(output.num_iters < input.max_iter) )
      output.term_flag = 0;
  } /* end while loop */
  /*
   *  Finish computing the standard error estimates vector se.
   */
  double temp = 1.0;
  if( input.num_rows > input.num_cols )
    temp = ( double ) ( input.num_rows - input.num_cols );
  if( Sqr(input.damp_val) > 0.0 )
    temp = ( double ) ( input.num_rows ); 
  temp = output.resid_norm / Sqrt( temp );
  
  for(int i = 0; i < input.num_cols; i++)
    /* update the standard error estimates vector se */
    output.std_err(i) = temp * Sqrt( output.std_err(i) );
  /*
   *  If statistics are printed at each iteration, print the statistics for the
   *  stopping condition.
   */
  if( input.lsqr_fp_out != NULL ) {
    fprintf( input.lsqr_fp_out, "\n\tISTOP = %3i\t\t\tITER = %9i\n\
	|| A ||_F = %13.5e\tcond( A ) = %13.5e\n\
	|| r ||_2 = %13.5e\t|| A^T r ||_2 = %13.5e\n\
	|| b ||_2 = %13.5e\t|| x - x0 ||_2 = %13.5e\n\n", 
	     output.term_flag, output.num_iters, output.frob_mat_norm, 
	     output.mat_cond_num, output.resid_norm, output.mat_resid_norm,
	     bnorm, output.sol_norm );

    fprintf( input.lsqr_fp_out, "  %s\n\n", lsqr_term_msg[output.term_flag]);
      
  }

  return;
}


