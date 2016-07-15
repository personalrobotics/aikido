/* glplpx02.c */

/***********************************************************************
*  This code is part of GLPK (GNU Linear Programming Kit).
*
*  Copyright (C) 2000,01,02,03,04,05,06,07,08,2009 Andrew Makhorin,
*  Department for Applied Informatics, Moscow Aviation Institute,
*  Moscow, Russia. All rights reserved. E-mail: <mao@mai2.rcnet.ru>.
*
*  GLPK is free software: you can redistribute it and/or modify it
*  under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  GLPK is distributed in the hope that it will be useful, but WITHOUT
*  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
*  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
*  License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with GLPK. If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/

#include "glpapi.h"
#include "glplib.h"
#define xfault xerror
#define lpx_get_b_info glp_get_bhead
#define lpx_get_row_b_ind glp_get_row_bind
#define lpx_get_col_b_ind glp_get_col_bind
#define lpx_ftran glp_ftran
#define lpx_btran glp_btran

/*----------------------------------------------------------------------
-- lpx_order_matrix - order rows and columns of the constraint matrix.
--
-- *Synopsis*
--
-- #include "glplpx.h"
-- void lpx_order_matrix(glp_prob *lp);
--
-- *Description*
--
-- The routine lpx_order_matrix rebuilds row and column linked lists of
-- the constraint matrix of the specified problem object.
--
-- On exit the constraint matrix is not changed, however, elements in
-- the row linked lists are ordered in ascending their column indices,
-- and elements in the column linked are ordered in ascending their row
-- indices. */

void lpx_order_matrix(glp_prob *lp)
{     GLPAIJ *aij;
      int i, j;
      /* rebuild row lists */
      for (i = lp->m; i >= 1; i--)
         lp->row[i]->ptr = NULL;
      for (j = lp->n; j >= 1; j--)
      {  for (aij = lp->col[j]->ptr; aij != NULL; aij = aij->c_next)
         {  i = aij->row->i;
            aij->r_prev = NULL;
            aij->r_next = lp->row[i]->ptr;
            if (aij->r_next != NULL) aij->r_next->r_prev = aij;
            lp->row[i]->ptr = aij;
         }
      }
      /* rebuild column lists */
      for (j = lp->n; j >= 1; j--)
         lp->col[j]->ptr = NULL;
      for (i = lp->m; i >= 1; i--)
      {  for (aij = lp->row[i]->ptr; aij != NULL; aij = aij->r_next)
         {  j = aij->col->j;
            aij->c_prev = NULL;
            aij->c_next = lp->col[j]->ptr;
            if (aij->c_next != NULL) aij->c_next->c_prev = aij;
            lp->col[j]->ptr = aij;
         }
      }
      return;
}

/***********************************************************************
*  NAME
*
*  lpx_put_solution - store basic solution components
*
*  SYNOPSIS
*
*  void lpx_put_solution(glp_prob *lp, int inval, const int *p_stat,
*     const int *d_stat, const double *obj_val, const int r_stat[],
*     const double r_prim[], const double r_dual[], const int c_stat[],
*     const double c_prim[], const double c_dual[])
*
*  DESCRIPTION
*
*  The routine lpx_put_solution stores basic solution components to the
*  specified problem object.
*
*  The parameter inval is the basis factorization invalidity flag.
*  If this flag is clear, the current status of the basis factorization
*  remains unchanged. If this flag is set, the routine invalidates the
*  basis factorization.
*
*  The parameter p_stat is a pointer to the status of primal basic
*  solution, which should be specified as follows:
*
*  GLP_UNDEF  - primal solution is undefined;
*  GLP_FEAS   - primal solution is feasible;
*  GLP_INFEAS - primal solution is infeasible;
*  GLP_NOFEAS - no primal feasible solution exists.
*
*  If the parameter p_stat is NULL, the current status of primal basic
*  solution remains unchanged.
*
*  The parameter d_stat is a pointer to the status of dual basic
*  solution, which should be specified as follows:
*
*  GLP_UNDEF  - dual solution is undefined;
*  GLP_FEAS   - dual solution is feasible;
*  GLP_INFEAS - dual solution is infeasible;
*  GLP_NOFEAS - no dual feasible solution exists.
*
*  If the parameter d_stat is NULL, the current status of dual basic
*  solution remains unchanged.
*
*  The parameter obj_val is a pointer to the objective function value.
*  If it is NULL, the current value of the objective function remains
*  unchanged.
*
*  The array element r_stat[i], 1 <= i <= m (where m is the number of
*  rows in the problem object), specifies the status of i-th auxiliary
*  variable, which should be specified as follows:
*
*  GLP_BS - basic variable;
*  GLP_NL - non-basic variable on lower bound;
*  GLP_NU - non-basic variable on upper bound;
*  GLP_NF - non-basic free variable;
*  GLP_NS - non-basic fixed variable.
*
*  If the parameter r_stat is NULL, the current statuses of auxiliary
*  variables remain unchanged.
*
*  The array element r_prim[i], 1 <= i <= m (where m is the number of
*  rows in the problem object), specifies a primal value of i-th
*  auxiliary variable. If the parameter r_prim is NULL, the current
*  primal values of auxiliary variables remain unchanged.
*
*  The array element r_dual[i], 1 <= i <= m (where m is the number of
*  rows in the problem object), specifies a dual value (reduced cost)
*  of i-th auxiliary variable. If the parameter r_dual is NULL, the
*  current dual values of auxiliary variables remain unchanged.
*
*  The array element c_stat[j], 1 <= j <= n (where n is the number of
*  columns in the problem object), specifies the status of j-th
*  structural variable, which should be specified as follows:
*
*  GLP_BS - basic variable;
*  GLP_NL - non-basic variable on lower bound;
*  GLP_NU - non-basic variable on upper bound;
*  GLP_NF - non-basic free variable;
*  GLP_NS - non-basic fixed variable.
*
*  If the parameter c_stat is NULL, the current statuses of structural
*  variables remain unchanged.
*
*  The array element c_prim[j], 1 <= j <= n (where n is the number of
*  columns in the problem object), specifies a primal value of j-th
*  structural variable. If the parameter c_prim is NULL, the current
*  primal values of structural variables remain unchanged.
*
*  The array element c_dual[j], 1 <= j <= n (where n is the number of
*  columns in the problem object), specifies a dual value (reduced cost)
*  of j-th structural variable. If the parameter c_dual is NULL, the
*  current dual values of structural variables remain unchanged. */

void lpx_put_solution(glp_prob *lp, int inval, const int *p_stat,
      const int *d_stat, const double *obj_val, const int r_stat[],
      const double r_prim[], const double r_dual[], const int c_stat[],
      const double c_prim[], const double c_dual[])
{     GLPROW *row;
      GLPCOL *col;
      int i, j;
      /* invalidate the basis factorization, if required */
      if (inval) lp->valid = 0;
      /* store primal status */
      if (p_stat != NULL)
      {  if (!(*p_stat == GLP_UNDEF  || *p_stat == GLP_FEAS ||
               *p_stat == GLP_INFEAS || *p_stat == GLP_NOFEAS))
            xfault("lpx_put_solution: p_stat = %d; invalid primal statu"
               "s\n", *p_stat);
         lp->pbs_stat = *p_stat;
      }
      /* store dual status */
      if (d_stat != NULL)
      {  if (!(*d_stat == GLP_UNDEF  || *d_stat == GLP_FEAS ||
               *d_stat == GLP_INFEAS || *d_stat == GLP_NOFEAS))
            xfault("lpx_put_solution: d_stat = %d; invalid dual status "
               "\n", *d_stat);
         lp->dbs_stat = *d_stat;
      }
      /* store objective function value */
      if (obj_val != NULL) lp->obj_val = *obj_val;
      /* store row solution components */
      for (i = 1; i <= lp->m; i++)
      {  row = lp->row[i];
         if (r_stat != NULL)
         {  if (!(r_stat[i] == GLP_BS ||
                  row->type == GLP_FR && r_stat[i] == GLP_NF ||
                  row->type == GLP_LO && r_stat[i] == GLP_NL ||
                  row->type == GLP_UP && r_stat[i] == GLP_NU ||
                  row->type == GLP_DB && r_stat[i] == GLP_NL ||
                  row->type == GLP_DB && r_stat[i] == GLP_NU ||
                  row->type == GLP_FX && r_stat[i] == GLP_NS))
               xfault("lpx_put_solution: r_stat[%d] = %d; invalid row s"
                  "tatus\n", i, r_stat[i]);
            row->stat = r_stat[i];
         }
         if (r_prim != NULL) row->prim = r_prim[i];
         if (r_dual != NULL) row->dual = r_dual[i];
      }
      /* store column solution components */
      for (j = 1; j <= lp->n; j++)
      {  col = lp->col[j];
         if (c_stat != NULL)
         {  if (!(c_stat[j] == GLP_BS ||
                  col->type == GLP_FR && c_stat[j] == GLP_NF ||
                  col->type == GLP_LO && c_stat[j] == GLP_NL ||
                  col->type == GLP_UP && c_stat[j] == GLP_NU ||
                  col->type == GLP_DB && c_stat[j] == GLP_NL ||
                  col->type == GLP_DB && c_stat[j] == GLP_NU ||
                  col->type == GLP_FX && c_stat[j] == GLP_NS))
               xfault("lpx_put_solution: c_stat[%d] = %d; invalid colum"
                  "n status\n", j, c_stat[j]);
            col->stat = c_stat[j];
         }
         if (c_prim != NULL) col->prim = c_prim[j];
         if (c_dual != NULL) col->dual = c_dual[j];
      }
      return;
}

/*----------------------------------------------------------------------
-- lpx_put_mip_soln - store mixed integer solution components.
--
-- *Synopsis*
--
-- #include "glplpx.h"
-- void lpx_put_mip_soln(glp_prob *lp, int i_stat, double row_mipx[],
--    double col_mipx[]);
--
-- *Description*
--
-- The routine lpx_put_mip_soln stores solution components obtained by
-- branch-and-bound solver into the specified problem object.
--
-- NOTE: This routine is intended for internal use only. */

void lpx_put_mip_soln(glp_prob *lp, int i_stat, double row_mipx[],
      double col_mipx[])
{     GLPROW *row;
      GLPCOL *col;
      int i, j;
      double sum;
      /* store mixed integer status */
#if 0
      if (!(i_stat == LPX_I_UNDEF || i_stat == LPX_I_OPT ||
            i_stat == LPX_I_FEAS  || i_stat == LPX_I_NOFEAS))
         fault("lpx_put_mip_soln: i_stat = %d; invalid mixed integer st"
            "atus", i_stat);
      lp->i_stat = i_stat;
#else
      switch (i_stat)
      {  case LPX_I_UNDEF:
            lp->mip_stat = GLP_UNDEF; break;
         case LPX_I_OPT:
            lp->mip_stat = GLP_OPT;  break;
         case LPX_I_FEAS:
            lp->mip_stat = GLP_FEAS; break;
         case LPX_I_NOFEAS:
            lp->mip_stat = GLP_NOFEAS; break;
         default:
            xfault("lpx_put_mip_soln: i_stat = %d; invalid mixed intege"
               "r status\n", i_stat);
      }
#endif
      /* store row solution components */
      if (row_mipx != NULL)
      {  for (i = 1; i <= lp->m; i++)
         {  row = lp->row[i];
            row->mipx = row_mipx[i];
         }
      }
      /* store column solution components */
      if (col_mipx != NULL)
      {  for (j = 1; j <= lp->n; j++)
         {  col = lp->col[j];
            col->mipx = col_mipx[j];
         }
      }
      /* if the solution is claimed to be integer feasible, check it */
      if (lp->mip_stat == GLP_OPT || lp->mip_stat == GLP_FEAS)
      {  for (j = 1; j <= lp->n; j++)
         {  col = lp->col[j];
            if (col->kind == GLP_IV && col->mipx != floor(col->mipx))
               xfault("lpx_put_mip_soln: col_mipx[%d] = %.*g; must be i"
                  "ntegral\n", j, DBL_DIG, col->mipx);
         }
      }
      /* compute the objective function value */
      sum = lp->c0;
      for (j = 1; j <= lp->n; j++)
      {  col = lp->col[j];
         sum += col->coef * col->mipx;
      }
      lp->mip_obj = sum;
      return;
}

/*----------------------------------------------------------------------
-- lpx_transform_row - transform explicitly specified row.
--
-- *Synopsis*
--
-- #include "glplpx.h"
-- int lpx_transform_row(LPX *lp, int len, int ind[], double val[]);
--
-- *Description*
--
-- The routine lpx_transform_row performs the same operation as the
-- routine lpx_eval_tab_row with exception that the transformed row is
-- specified explicitly as a sparse vector.
--
-- The explicitly specified row may be thought as a linear form:
--
--    x = a[1]*x[m+1] + a[2]*x[m+2] + ... + a[n]*x[m+n],             (1)
--
-- where x is an auxiliary variable for this row, a[j] are coefficients
-- of the linear form, x[m+j] are structural variables.
--
-- On entry column indices and numerical values of non-zero elements of
-- the row should be stored in locations ind[1], ..., ind[len] and
-- val[1], ..., val[len], where len is the number of non-zero elements.
--
-- This routine uses the system of equality constraints and the current
-- basis in order to express the auxiliary variable x in (1) through the
-- current non-basic variables (as if the transformed row were added to
-- the problem object and its auxiliary variable were basic), i.e. the
-- resultant row has the form:
--
--    x = alfa[1]*xN[1] + alfa[2]*xN[2] + ... + alfa[n]*xN[n],       (2)
--
-- where xN[j] are non-basic (auxiliary or structural) variables, n is
-- the number of columns in the LP problem object.
--
-- On exit the routine stores indices and numerical values of non-zero
-- elements of the resultant row (2) in locations ind[1], ..., ind[len']
-- and val[1], ..., val[len'], where 0 <= len' <= n is the number of
-- non-zero elements in the resultant row returned by the routine. Note
-- that indices (numbers) of non-basic variables stored in the array ind
-- correspond to original ordinal numbers of variables: indices 1 to m
-- mean auxiliary variables and indices m+1 to m+n mean structural ones.
--
-- *Returns*
--
-- The routine returns len', which is the number of non-zero elements in
-- the resultant row stored in the arrays ind and val.
--
-- *Background*
--
-- The explicitly specified row (1) is transformed in the same way as
-- it were the objective function row.
--
-- From (1) it follows that:
--
--    x = aB * xB + aN * xN,                                         (3)
--
-- where xB is the vector of basic variables, xN is the vector of
-- non-basic variables.
--
-- The simplex table, which corresponds to the current basis, is:
--
--    xB = [-inv(B) * N] * xN.                                       (4)
--
-- Therefore substituting xB from (4) to (3) we have:
--
--    x = aB * [-inv(B) * N] * xN + aN * xN =
--                                                                   (5)
--      = rho * (-N) * xN + aN * xN = alfa * xN,
--
-- where:
--
--    rho = inv(B') * aB,                                            (6)
--
-- and
--
--    alfa = aN + rho * (-N)                                         (7)
--
-- is the resultant row computed by the routine. */

int lpx_transform_row(LPX *lp, int len, int ind[], double val[])
{     int i, j, k, m, n, t, lll, *iii;
      double alfa, *a, *aB, *rho, *vvv;
      if (!lpx_is_b_avail(lp))
         xfault("lpx_transform_row: LP basis is not available\n");
      m = lpx_get_num_rows(lp);
      n = lpx_get_num_cols(lp);
      /* unpack the row to be transformed to the array a */
      a = xcalloc(1+n, sizeof(double));
      for (j = 1; j <= n; j++) a[j] = 0.0;
      if (!(0 <= len && len <= n))
         xfault("lpx_transform_row: len = %d; invalid row length\n",
            len);
      for (t = 1; t <= len; t++)
      {  j = ind[t];
         if (!(1 <= j && j <= n))
            xfault("lpx_transform_row: ind[%d] = %d; column index out o"
               "f range\n", t, j);
         if (val[t] == 0.0)
            xfault("lpx_transform_row: val[%d] = 0; zero coefficient no"
               "t allowed\n", t);
         if (a[j] != 0.0)
            xfault("lpx_transform_row: ind[%d] = %d; duplicate column i"
               "ndices not allowed\n", t, j);
         a[j] = val[t];
      }
      /* construct the vector aB */
      aB = xcalloc(1+m, sizeof(double));
      for (i = 1; i <= m; i++)
      {  k = lpx_get_b_info(lp, i);
         /* xB[i] is k-th original variable */
         xassert(1 <= k && k <= m+n);
         aB[i] = (k <= m ? 0.0 : a[k-m]);
      }
      /* solve the system B'*rho = aB to compute the vector rho */
      rho = aB, lpx_btran(lp, rho);
      /* compute coefficients at non-basic auxiliary variables */
      len = 0;
      for (i = 1; i <= m; i++)
      {  if (lpx_get_row_stat(lp, i) != LPX_BS)
         {  alfa = - rho[i];
            if (alfa != 0.0)
            {  len++;
               ind[len] = i;
               val[len] = alfa;
            }
         }
      }
      /* compute coefficients at non-basic structural variables */
      iii = xcalloc(1+m, sizeof(int));
      vvv = xcalloc(1+m, sizeof(double));
      for (j = 1; j <= n; j++)
      {  if (lpx_get_col_stat(lp, j) != LPX_BS)
         {  alfa = a[j];
            lll = lpx_get_mat_col(lp, j, iii, vvv);
            for (t = 1; t <= lll; t++) alfa += vvv[t] * rho[iii[t]];
            if (alfa != 0.0)
            {  len++;
               ind[len] = m+j;
               val[len] = alfa;
            }
         }
      }
      xassert(len <= n);
      xfree(iii);
      xfree(vvv);
      xfree(aB);
      xfree(a);
      return len;
}

/*----------------------------------------------------------------------
-- lpx_transform_col - transform explicitly specified column.
--
-- *Synopsis*
--
-- #include "glplpx.h"
-- int lpx_transform_col(LPX *lp, int len, int ind[], double val[]);
--
-- *Description*
--
-- The routine lpx_transform_col performs the same operation as the
-- routine lpx_eval_tab_col with exception that the transformed column
-- is specified explicitly as a sparse vector.
--
-- The explicitly specified column may be thought as if it were added
-- to the original system of equality constraints:
--
--    x[1] = a[1,1]*x[m+1] + ... + a[1,n]*x[m+n] + a[1]*x
--    x[2] = a[2,1]*x[m+1] + ... + a[2,n]*x[m+n] + a[2]*x            (1)
--       .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
--    x[m] = a[m,1]*x[m+1] + ... + a[m,n]*x[m+n] + a[m]*x
--
-- where x[i] are auxiliary variables, x[m+j] are structural variables,
-- x is a structural variable for the explicitly specified column, a[i]
-- are constraint coefficients for x.
--
-- On entry row indices and numerical values of non-zero elements of
-- the column should be stored in locations ind[1], ..., ind[len] and
-- val[1], ..., val[len], where len is the number of non-zero elements.
--
-- This routine uses the system of equality constraints and the current
-- basis in order to express the current basic variables through the
-- structural variable x in (1) (as if the transformed column were added
-- to the problem object and the variable x were non-basic), i.e. the
-- resultant column has the form:
--
--    xB[1] = ... + alfa[1]*x
--    xB[2] = ... + alfa[2]*x                                        (2)
--       .  .  .  .  .  .
--    xB[m] = ... + alfa[m]*x
--
-- where xB are basic (auxiliary and structural) variables, m is the
-- number of rows in the problem object.
--
-- On exit the routine stores indices and numerical values of non-zero
-- elements of the resultant column (2) in locations ind[1], ...,
-- ind[len'] and val[1], ..., val[len'], where 0 <= len' <= m is the
-- number of non-zero element in the resultant column returned by the
-- routine. Note that indices (numbers) of basic variables stored in
-- the array ind correspond to original ordinal numbers of variables:
-- indices 1 to m mean auxiliary variables and indices m+1 to m+n mean
-- structural ones.
--
-- *Returns*
--
-- The routine returns len', which is the number of non-zero elements
-- in the resultant column stored in the arrays ind and val.
--
-- *Background*
--
-- The explicitly specified column (1) is transformed in the same way
-- as any other column of the constraint matrix using the formula:
--
--    alfa = inv(B) * a,                                             (3)
--
-- where alfa is the resultant column computed by the routine. */

int lpx_transform_col(LPX *lp, int len, int ind[], double val[])
{     int i, m, t;
      double *a, *alfa;
      if (!lpx_is_b_avail(lp))
         xfault("lpx_transform_col: LP basis is not available\n");
      m = lpx_get_num_rows(lp);
      /* unpack the column to be transformed to the array a */
      a = xcalloc(1+m, sizeof(double));
      for (i = 1; i <= m; i++) a[i] = 0.0;
      if (!(0 <= len && len <= m))
         xfault("lpx_transform_col: len = %d; invalid column length\n",
            len);
      for (t = 1; t <= len; t++)
      {  i = ind[t];
         if (!(1 <= i && i <= m))
            xfault("lpx_transform_col: ind[%d] = %d; row index out of r"
               "ange\n", t, i);
         if (val[t] == 0.0)
            xfault("lpx_transform_col: val[%d] = 0; zero coefficient no"
               "t allowed\n", t);
         if (a[i] != 0.0)
            xfault("lpx_transform_col: ind[%d] = %d; duplicate row indi"
               "ces not allowed\n", t, i);
         a[i] = val[t];
      }
      /* solve the system B*a = alfa to compute the vector alfa */
      alfa = a, lpx_ftran(lp, alfa);
      /* store resultant coefficients */
      len = 0;
      for (i = 1; i <= m; i++)
      {  if (alfa[i] != 0.0)
         {  len++;
            ind[len] = lpx_get_b_info(lp, i);
            val[len] = alfa[i];
         }
      }
      xfree(a);
      return len;
}

/*----------------------------------------------------------------------
-- lpx_prim_ratio_test - perform primal ratio test.
--
-- *Synopsis*
--
-- #include "glplpx.h"
-- int lpx_prim_ratio_test(LPX *lp, int len, int ind[], double val[],
--    int how, double tol);
--
-- *Description*
--
-- The routine lpx_prim_ratio_test performs the primal ratio test for
-- an explicitly specified column of the simplex table.
--
-- The primal basic solution associated with an LP problem object,
-- which the parameter lp points to, should be feasible. No components
-- of the LP problem object are changed by the routine.
--
-- The explicitly specified column of the simplex table shows how the
-- basic variables xB depend on some non-basic variable y (which is not
-- necessarily presented in the problem object):
--
--    xB[1] = ... + alfa[1]*y + ...
--    xB[2] = ... + alfa[2]*y + ...                                  (*)
--       .  .  .  .  .  .  .  .
--    xB[m] = ... + alfa[m]*y + ...
--
-- The column (*) is specifed on entry to the routine using the sparse
-- format. Ordinal numbers of basic variables xB[i] should be placed in
-- locations ind[1], ..., ind[len], where ordinal number 1 to m denote
-- auxiliary variables, and ordinal numbers m+1 to m+n denote structural
-- variables. The corresponding non-zero coefficients alfa[i] should be
-- placed in locations val[1], ..., val[len]. The arrays ind and val are
-- not changed on exit.
--
-- The parameter how specifies in which direction the variable y changes
-- on entering the basis: +1 means increasing, -1 means decreasing.
--
-- The parameter tol is a relative tolerance (small positive number)
-- used by the routine to skip small alfa[i] of the column (*).
--
-- The routine determines the ordinal number of some basic variable
-- (specified in ind[1], ..., ind[len]), which should leave the basis
-- instead the variable y in order to keep primal feasibility, and
-- returns it on exit. If the choice cannot be made (i.e. if the
-- adjacent basic solution is primal unbounded), the routine returns
-- zero.
--
-- *Note*
--
-- If the non-basic variable y is presented in the LP problem object,
-- the column (*) can be computed using the routine lpx_eval_tab_col.
-- Otherwise it can be computed using the routine lpx_transform_col.
--
-- *Returns*
--
-- The routine lpx_prim_ratio_test returns the ordinal number of some
-- basic variable xB[i], which should leave the basis instead the
-- variable y in order to keep primal feasibility. If the adjacent basic
-- solution is primal unbounded and therefore the choice cannot be made,
-- the routine returns zero. */

int lpx_prim_ratio_test(LPX *lp, int len, const int ind[],
      const double val[], int how, double tol)
{     int i, k, m, n, p, t, typx, tagx;
      double alfa_i, abs_alfa_i, big, eps, bbar_i, lb_i, ub_i, temp,
         teta;
      if (!lpx_is_b_avail(lp))
         xfault("lpx_prim_ratio_test: LP basis is not available\n");
      if (lpx_get_prim_stat(lp) != LPX_P_FEAS)
         xfault("lpx_prim_ratio_test: current basic solution is not pri"
            "mal feasible\n");
      if (!(how == +1 || how == -1))
         xfault("lpx_prim_ratio_test: how = %d; invalid parameter\n",
            how);
      m = lpx_get_num_rows(lp);
      n = lpx_get_num_cols(lp);
      /* compute the largest absolute value of the specified influence
         coefficients */
      big = 0.0;
      for (t = 1; t <= len; t++)
      {  temp = val[t];
         if (temp < 0.0) temp = - temp;
         if (big < temp) big = temp;
      }
      /* compute the absolute tolerance eps used to skip small entries
         of the column */
      if (!(0.0 < tol && tol < 1.0))
         xfault("lpx_prim_ratio_test: tol = %g; invalid tolerance\n",
            tol);
      eps = tol * (1.0 + big);
      /* initial settings */
      p = 0, teta = DBL_MAX, big = 0.0;
      /* walk through the entries of the specified column */
      for (t = 1; t <= len; t++)
      {  /* get the ordinal number of basic variable */
         k = ind[t];
         if (!(1 <= k && k <= m+n))
            xfault("lpx_prim_ratio_test: ind[%d] = %d; variable number "
               "out of range\n", t, k);
         if (k <= m)
            tagx = lpx_get_row_stat(lp, k);
         else
            tagx = lpx_get_col_stat(lp, k-m);
         if (tagx != LPX_BS)
            xfault("lpx_prim_ratio_test: ind[%d] = %d; non-basic variab"
               "le not allowed\n", t, k);
         /* determine index of the variable x[k] in the vector xB */
         if (k <= m)
            i = lpx_get_row_b_ind(lp, k);
         else
            i = lpx_get_col_b_ind(lp, k-m);
         xassert(1 <= i && i <= m);
         /* determine unscaled bounds and value of the basic variable
            xB[i] in the current basic solution */
         if (k <= m)
         {  typx = lpx_get_row_type(lp, k);
            lb_i = lpx_get_row_lb(lp, k);
            ub_i = lpx_get_row_ub(lp, k);
            bbar_i = lpx_get_row_prim(lp, k);
         }
         else
         {  typx = lpx_get_col_type(lp, k-m);
            lb_i = lpx_get_col_lb(lp, k-m);
            ub_i = lpx_get_col_ub(lp, k-m);
            bbar_i = lpx_get_col_prim(lp, k-m);
         }
         /* determine influence coefficient for the basic variable
            x[k] = xB[i] in the explicitly specified column and turn to
            the case of increasing the variable y in order to simplify
            the program logic */
         alfa_i = (how > 0 ? +val[t] : -val[t]);
         abs_alfa_i = (alfa_i > 0.0 ? +alfa_i : -alfa_i);
         /* analyze main cases */
         switch (typx)
         {  case LPX_FR:
               /* xB[i] is free variable */
               continue;
            case LPX_LO:
lo:            /* xB[i] has an lower bound */
               if (alfa_i > - eps) continue;
               temp = (lb_i - bbar_i) / alfa_i;
               break;
            case LPX_UP:
up:            /* xB[i] has an upper bound */
               if (alfa_i < + eps) continue;
               temp = (ub_i - bbar_i) / alfa_i;
               break;
            case LPX_DB:
               /* xB[i] has both lower and upper bounds */
               if (alfa_i < 0.0) goto lo; else goto up;
            case LPX_FX:
               /* xB[i] is fixed variable */
               if (abs_alfa_i < eps) continue;
               temp = 0.0;
               break;
            default:
               xassert(typx != typx);
         }
         /* if the value of the variable xB[i] violates its lower or
            upper bound (slightly, because the current basis is assumed
            to be primal feasible), temp is negative; we can think this
            happens due to round-off errors and the value is exactly on
            the bound; this allows replacing temp by zero */
         if (temp < 0.0) temp = 0.0;
         /* apply the minimal ratio test */
         if (teta > temp || teta == temp && big < abs_alfa_i)
            p = k, teta = temp, big = abs_alfa_i;
      }
      /* return the ordinal number of the chosen basic variable */
      return p;
}

/*----------------------------------------------------------------------
-- lpx_dual_ratio_test - perform dual ratio test.
--
-- *Synopsis*
--
-- #include "glplpx.h"
-- int lpx_dual_ratio_test(LPX *lp, int len, int ind[], double val[],
--    int how, double tol);
--
-- *Description*
--
-- The routine lpx_dual_ratio_test performs the dual ratio test for an
-- explicitly specified row of the simplex table.
--
-- The dual basic solution associated with an LP problem object, which
-- the parameter lp points to, should be feasible. No components of the
-- LP problem object are changed by the routine.
--
-- The explicitly specified row of the simplex table is a linear form,
-- which shows how some basic variable y (not necessarily presented in
-- the problem object) depends on non-basic variables xN:
--
--    y = alfa[1]*xN[1] + alfa[2]*xN[2] + ... + alfa[n]*xN[n].       (*)
--
-- The linear form (*) is specified on entry to the routine using the
-- sparse format. Ordinal numbers of non-basic variables xN[j] should be
-- placed in locations ind[1], ..., ind[len], where ordinal numbers 1 to
-- m denote auxiliary variables, and ordinal numbers m+1 to m+n denote
-- structural variables. The corresponding non-zero coefficients alfa[j]
-- should be placed in locations val[1], ..., val[len]. The arrays ind
-- and val are not changed on exit.
--
-- The parameter how specifies in which direction the variable y changes
-- on leaving the basis: +1 means increasing, -1 means decreasing.
--
-- The parameter tol is a relative tolerance (small positive number)
-- used by the routine to skip small alfa[j] of the form (*).
--
-- The routine determines the ordinal number of some non-basic variable
-- (specified in ind[1], ..., ind[len]), which should enter the basis
-- instead the variable y in order to keep dual feasibility, and returns
-- it on exit. If the choice cannot be made (i.e. if the adjacent basic
-- solution is dual unbounded), the routine returns zero.
--
-- *Note*
--
-- If the basic variable y is presented in the LP problem object, the
-- row (*) can be computed using the routine lpx_eval_tab_row. Otherwise
-- it can be computed using the routine lpx_transform_row.
--
-- *Returns*
--
-- The routine lpx_dual_ratio_test returns the ordinal number of some
-- non-basic variable xN[j], which should enter the basis instead the
-- variable y in order to keep dual feasibility. If the adjacent basic
-- solution is dual unbounded and therefore the choice cannot be made,
-- the routine returns zero. */

int lpx_dual_ratio_test(LPX *lp, int len, const int ind[],
      const double val[], int how, double tol)
{     int k, m, n, t, q, tagx;
      double dir, alfa_j, abs_alfa_j, big, eps, cbar_j, temp, teta;
      if (!lpx_is_b_avail(lp))
         xfault("lpx_dual_ratio_test: LP basis is not available\n");
      if (lpx_get_dual_stat(lp) != LPX_D_FEAS)
         xfault("lpx_dual_ratio_test: current basic solution is not dua"
            "l feasible\n");
      if (!(how == +1 || how == -1))
         xfault("lpx_dual_ratio_test: how = %d; invalid parameter\n",
            how);
      m = lpx_get_num_rows(lp);
      n = lpx_get_num_cols(lp);
      dir = (lpx_get_obj_dir(lp) == LPX_MIN ? +1.0 : -1.0);
      /* compute the largest absolute value of the specified influence
         coefficients */
      big = 0.0;
      for (t = 1; t <= len; t++)
      {  temp = val[t];
         if (temp < 0.0) temp = - temp;
         if (big < temp) big = temp;
      }
      /* compute the absolute tolerance eps used to skip small entries
         of the row */
      if (!(0.0 < tol && tol < 1.0))
         xfault("lpx_dual_ratio_test: tol = %g; invalid tolerance\n",
            tol);
      eps = tol * (1.0 + big);
      /* initial settings */
      q = 0, teta = DBL_MAX, big = 0.0;
      /* walk through the entries of the specified row */
      for (t = 1; t <= len; t++)
      {  /* get ordinal number of non-basic variable */
         k = ind[t];
         if (!(1 <= k && k <= m+n))
            xfault("lpx_dual_ratio_test: ind[%d] = %d; variable number "
               "out of range\n", t, k);
         if (k <= m)
            tagx = lpx_get_row_stat(lp, k);
         else
            tagx = lpx_get_col_stat(lp, k-m);
         if (tagx == LPX_BS)
            xfault("lpx_dual_ratio_test: ind[%d] = %d; basic variable n"
               "ot allowed\n", t, k);
         /* determine unscaled reduced cost of the non-basic variable
            x[k] = xN[j] in the current basic solution */
         if (k <= m)
            cbar_j = lpx_get_row_dual(lp, k);
         else
            cbar_j = lpx_get_col_dual(lp, k-m);
         /* determine influence coefficient at the non-basic variable
            x[k] = xN[j] in the explicitly specified row and turn to
            the case of increasing the variable y in order to simplify
            program logic */
         alfa_j = (how > 0 ? +val[t] : -val[t]);
         abs_alfa_j = (alfa_j > 0.0 ? +alfa_j : -alfa_j);
         /* analyze main cases */
         switch (tagx)
         {  case LPX_NL:
               /* xN[j] is on its lower bound */
               if (alfa_j < +eps) continue;
               temp = (dir * cbar_j) / alfa_j;
               break;
            case LPX_NU:
               /* xN[j] is on its upper bound */
               if (alfa_j > -eps) continue;
               temp = (dir * cbar_j) / alfa_j;
               break;
            case LPX_NF:
               /* xN[j] is non-basic free variable */
               if (abs_alfa_j < eps) continue;
               temp = 0.0;
               break;
            case LPX_NS:
               /* xN[j] is non-basic fixed variable */
               continue;
            default:
               xassert(tagx != tagx);
         }
         /* if the reduced cost of the variable xN[j] violates its zero
            bound (slightly, because the current basis is assumed to be
            dual feasible), temp is negative; we can think this happens
            due to round-off errors and the reduced cost is exact zero;
            this allows replacing temp by zero */
         if (temp < 0.0) temp = 0.0;
         /* apply the minimal ratio test */
         if (teta > temp || teta == temp && big < abs_alfa_j)
            q = k, teta = temp, big = abs_alfa_j;
      }
      /* return the ordinal number of the chosen non-basic variable */
      return q;
}

/* eof */
