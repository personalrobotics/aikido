/* glpnpp02.c */

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

#include "glpnpp.h"

/***********************************************************************
*  FREE ROW
*
*  Let row p be free, i.e. have no bounds:
*
*     -inf < sum a[p,j] * x[j] < +inf.
*
*  This "constraint" can never be active, so the free row is redundant
*  and therefore can be removed from the problem. */

struct free_row
{     /* free row */
      int p;
      /* reference number of the row */
      NPPLFE *ptr;
      /* list of non-zero coefficients a[p,j] */
};

static void rcv_free_row(NPP *npp, void *info);

void npp_free_row(NPP *npp, NPPROW *row)
{     /* process free row */
      struct free_row *info;
      NPPAIJ *aij;
      NPPLFE *lfe;
      /* the row must be free */
      xassert(row->lb == -DBL_MAX && row->ub == +DBL_MAX);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_free_row, sizeof(struct free_row));
      info->p = row->i;
      info->ptr = NULL;
      /* save the row coefficients */
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  lfe = dmp_get_atom(npp->stack, sizeof(NPPLFE));
         lfe->ref = aij->col->j;
         lfe->val = aij->val;
         lfe->next = info->ptr;
         info->ptr = lfe;
      }
      /* remove the row from the problem */
      npp_del_row(npp, row);
      return;
}

static void rcv_free_row(NPP *npp, void *_info)
{     /* recover free row */
      struct free_row *info = _info;
      NPPLFE *lfe;
      double sum;
      if (npp->sol == GLP_SOL)
      {  /* the free row is non-active */
         npp->r_stat[info->p] = GLP_BS;
      }
      /* compute auxiliary variable x[p] = sum a[p,j] * x[j] */
      sum = 0.0;
      for (lfe = info->ptr; lfe != NULL; lfe = lfe->next)
         sum += lfe->val * npp->c_prim[lfe->ref];
      npp->r_prim[info->p] = sum;
      /* reduced cost of x[p] is obviously zero */
      if (npp->sol != GLP_MIP)
         npp->r_dual[info->p] = 0.0;
      return;
}

/***********************************************************************
*  ROW OF 'GREATER THAN OR EQUAL TO' TYPE
*
*  Let row p be a 'greater than or equal to' inequality constraint:
*
*     L[p] <= sum a[p,j] * x[j] (<= U[p]).
*
*  Then it can be converted to equality constraint as follows:
*
*     sum a[p,j] * x[j] - s = L[p],
*
*     0 <= s (<= U[p] - L[p]),
*
*  where s is a surplus variable. */

struct ineq_row
{     /* inequality constraint row */
      int p;
      /* reference number of the row */
      int s;
      /* reference number of slack/surplus variable */
};

static void rcv_gteq_row(NPP *npp, void *info);

void npp_gteq_row(NPP *npp, NPPROW *row)
{     /* process row of 'greater than or equal to' type */
      struct ineq_row *info;
      NPPCOL *col;
      /* the row must have lower bound */
      xassert(row->lb != -DBL_MAX);
      /* create surplus variable */
      col = npp_add_col(npp);
      col->lb = 0.0;
      col->ub = (row->ub == +DBL_MAX ? +DBL_MAX : row->ub - row->lb);
      /* and add it to the transformed problem */
      npp_add_aij(npp, row, col, -1.0);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_gteq_row, sizeof(struct ineq_row));
      info->p = row->i;
      info->s = col->j;
      /* convert the row to equality constraint */
      row->ub = row->lb;
      return;
}

static void rcv_gteq_row(NPP *npp, void *_info)
{     /* recover row of 'greater than or equal to' type */
      struct ineq_row *info = _info;
      if (npp->sol == GLP_SOL)
      {  if (npp->r_stat[info->p] == GLP_BS)
            /* degenerate case; x[p] remains basic */;
         else if (npp->c_stat[info->s] == GLP_BS)
            npp->r_stat[info->p] = GLP_BS;
         else if (npp->c_stat[info->s] == GLP_NL)
            npp->r_stat[info->p] = GLP_NL;
         else if (npp->c_stat[info->s] == GLP_NU)
            npp->r_stat[info->p] = GLP_NU;
         else
            xassert(npp != npp);
      }
      /* compute auxiliary variable x[p] = sum a[p,j] * x[j] =
         = (sum a[p,j] * x[j] - s) + s = x'[p] + s */
      npp->r_prim[info->p] += npp->c_prim[info->s];
      /* determine reduced cost of x[p] */
      if (npp->sol != GLP_MIP)
         npp->r_dual[info->p] = + npp->c_dual[info->s];
      return;
}

/***********************************************************************
*  ROW OF 'LESS THAN OR EQUAL TO' TYPE
*
*  Let row p be a 'less than or equal to' inequality constraint:
*
*     (L[p] <=) sum a[p,j] * x[j] <= U[p].
*
*  Then it can be converted to equality constraint as follows:
*
*     sum a[p,j] * x[j] + s = U[p],
*
*     0 <= s (<= U[p] - L[p]),
*
*  where s is a slack variable. */

static void rcv_lteq_row(NPP *npp, void *info);

void npp_lteq_row(NPP *npp, NPPROW *row)
{     /* process row of 'less than or equal to' type */
      struct ineq_row *info;
      NPPCOL *col;
      /* the row must have upper bound */
      xassert(row->ub != +DBL_MAX);
      /* create slack variable */
      col = npp_add_col(npp);
      col->lb = 0.0;
      col->ub = (row->lb == -DBL_MAX ? +DBL_MAX : row->ub - row->lb);
      /* and add it to the transformed problem */
      npp_add_aij(npp, row, col, +1.0);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_lteq_row, sizeof(struct ineq_row));
      info->p = row->i;
      info->s = col->j;
      /* convert the row to equality constraint */
      row->lb = row->ub;
      return;
}

static void rcv_lteq_row(NPP *npp, void *_info)
{     /* recover row of 'less than or equal to' type */
      struct ineq_row *info = _info;
      if (npp->sol == GLP_SOL)
      {  if (npp->r_stat[info->p] == GLP_BS)
            /* degenerate case; x[p] remains basic */;
         else if (npp->c_stat[info->s] == GLP_BS)
            npp->r_stat[info->p] = GLP_BS;
         else if (npp->c_stat[info->s] == GLP_NL)
            npp->r_stat[info->p] = GLP_NU;
         else if (npp->c_stat[info->s] == GLP_NU)
            npp->r_stat[info->p] = GLP_NL;
         else
            xassert(npp != npp);
      }
      /* compute auxiliary variable x[p] = sum a[p,j] * x[j] =
         = (sum a[p,j] * x[j] + s) - s = x'[p] - s */
      npp->r_prim[info->p] -= npp->c_prim[info->s];
      /* determine reduced cost of x[p] */
      if (npp->sol != GLP_MIP)
         npp->r_dual[info->p] = - npp->c_dual[info->s];
      return;
}

/***********************************************************************
*  FREE COLUMN
*
*  Let column q be free:
*
*     -inf < x[q] < +inf.
*
*  Then it can be replaced by the following difference:
*
*     x[q] = x' - x''
*
*  where x', x'' are non-negative variables. */

struct free_col
{     /* free column */
      int q;
      /* reference number of column x[q] that becomes x' */
      int j;
      /* reference number of column x'' */
};

static void rcv_free_col(NPP *npp, void *info);

void npp_free_col(NPP *npp, NPPCOL *col)
{     /* process free column */
      struct free_col *info;
      NPPCOL *ccc;
      NPPAIJ *aij;
      /* the column must be free */
      xassert(col->lb == -DBL_MAX && col->ub == +DBL_MAX);
      /* variable x[q] becomes x' */
      col->lb = 0.0, col->ub = +DBL_MAX;
      /* create variable x'' */
      ccc = npp_add_col(npp);
      ccc->lb = 0.0, ccc->ub = +DBL_MAX;
      /* duplicate objective coefficient */
      ccc->coef = - col->coef;
      /* duplicate column of the constraint matrix */
      for (aij = col->ptr; aij != NULL; aij = aij->c_next)
         npp_add_aij(npp, aij->row, ccc, - aij->val);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_free_col, sizeof(struct free_col));
      info->q = col->j;
      info->j = ccc->j;
      return;
}

static void rcv_free_col(NPP *npp, void *_info)
{     /* recover free column */
      struct free_col *info = _info;
      if (npp->sol == GLP_SOL)
      {  if (npp->c_stat[info->q] == GLP_NL &&
             npp->c_stat[info->j] == GLP_NL)
         {  /* denegerate case; x[q] remains non-basic (super basic) */
            npp->c_stat[info->q] = GLP_NF;
         }
         else
         {  /* note that x' and x'' cannot be both basic due to linear
               dependence */
            npp->c_stat[info->q] = GLP_BS;
         }
      }
      /* compute variable x[q] = x' - x'' */
      npp->c_prim[info->q] -= npp->c_prim[info->j];
      /* since x[q] has no bounds, in any optimal and therefore dual
         feasible solution its reduced cost must be zero */
      if (npp->sol != GLP_MIP)
         npp->c_dual[info->q] = 0.0;
      return;
}

/***********************************************************************
*  COLUMN WITH LOWER BOUND
*
*  Let column q have lower bound:
*
*     l[q] <= x[q] (<= u[q]).
*
*  Then it can be substituted as follows:
*
*     x[q] = l[q] + x',
*
*  where 0 <= x' (<= u[q] - l[q]).
*
*  Substitution into the objective function gives:
*
*     z = sum c[j] * x[j] + c0 =
*          j
*
*       = sum c[j] * x[j] + c[q] * x[q] + c0 =
*         j!=q
*
*       = sum c[j] * x[j] + c[q] * (l[q] + x') + c0 =
*         j!=q
*
*       = sum c[j] * x[j] + c[q] * x' + (c0)',
*         j!=q
*
*  where (c0)' = c0 + c[q] * l[q].
*
*  Substitution into i-th row, 1 <= i <= m, a[i,q] != 0, gives:
*
*     L[i] <= sum a[i,j] * x[j] <= U[i]  ==>
*              j
*
*     L[i] <= sum a[i,j] * x[j] + a[i,q] * x[q] <= U[i]  ==>
*             j!=q
*
*     L[i] <= sum a[i,j] * x[j] + a[i,q] * (l[q] + x') <= U[i]  ==>
*             j!=q
*
*     L'[i] <= sum a[i,j] * x[j] + a[i,q] * x' <= U'[i],
*
*  where L'[i] = L[i] - a[i,q] * l[q], U'[i] = U[i] - a[i,q] * l[q]. */

struct bnd_col
{     /* bounded column */
      int q;
      /* reference number of column x[q] that becomes x' */
      double b;
      /* lower/upper bound of column x[q] */
      NPPLFE *ptr;
      /* list of non-zero coefficients a[i,q] */
};

static void rcv_lbnd_col(NPP *npp, void *info);

void npp_lbnd_col(NPP *npp, NPPCOL *col)
{     /* process column with lower bound */
      struct bnd_col *info;
      NPPROW *row;
      NPPAIJ *aij;
      NPPLFE *lfe;
      /* the column must have lower bound */
      xassert(col->lb != -DBL_MAX);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_lbnd_col, sizeof(struct bnd_col));
      info->q = col->j;
      info->b = col->lb;
      info->ptr = NULL;
      /* substitute x[q] into the objective function */
      npp->c0 += col->coef * col->lb;
      /* substitute x[q] into rows and save the column coefficients */
      for (aij = col->ptr; aij != NULL; aij = aij->c_next)
      {  row = aij->row;
         if (row->lb == row->ub)
            row->ub = (row->lb -= aij->val * col->lb);
         else
         {  if (row->lb != -DBL_MAX)
               row->lb -= aij->val * col->lb;
            if (row->ub != +DBL_MAX)
               row->ub -= aij->val * col->lb;
         }
         lfe = dmp_get_atom(npp->stack, sizeof(NPPLFE));
         lfe->ref = row->i;
         lfe->val = aij->val;
         lfe->next = info->ptr;
         info->ptr = lfe;
      }
      /* column x[q] becomes column x' */
      if (col->ub != +DBL_MAX)
         col->ub -= col->lb;
      col->lb = 0.0;
      return;
}

static void rcv_lbnd_col(NPP *npp, void *_info)
{     /* recover column with lower bound */
      struct bnd_col *info = _info;
      NPPLFE *lfe;
      if (npp->sol == GLP_SOL)
      {  /* x[q] has the same status as x' (NL, NU, or BS) */
         npp->c_stat[info->q] = npp->c_stat[info->q];
      }
      /* compute variable x[q] = l[q] + x' */
      npp->c_prim[info->q] = info->b + npp->c_prim[info->q];
      /* compute auxiliary variables x[i] = x'[i] + a[i,q] * l[q] */
      for (lfe = info->ptr; lfe != NULL; lfe = lfe->next)
         npp->r_prim[lfe->ref] += lfe->val * info->b;
      /* determine reduced cost of x[q] */
      if (npp->sol != GLP_MIP)
         npp->c_dual[info->q] = + npp->c_dual[info->q];
      return;
}

/***********************************************************************
*  COLUMN WITH UPPER BOUND
*
*  Let column q have upper bound:
*
*     (l[q] <=) x[q] <= u[q].
*
*  Then it can be substituted as follows:
*
*     x[q] = u[q] - x',
*
*  where 0 <= x' (<= u[q] - l[q]).
*
*  Substitution into the objective function gives:
*
*     z = sum c[j] * x[j] + c0 =
*          j
*
*       = sum c[j] * x[j] + c[q] * x[q] + c0 =
*         j!=q
*
*       = sum c[j] * x[j] + c[q] * (u[q] - x') + c0 =
*         j!=q
*
*       = sum c[j] * x[j] - c[q] * x' + (c0)',
*         j!=q
*
*  where (c0)' = c0 + c[q] * u[q].
*
*  Substitution into i-th row, 1 <= i <= m, a[i,q] != 0, gives:
*
*     L[i] <= sum a[i,j] * x[j] <= U[i]  ==>
*              j
*
*     L[i] <= sum a[i,j] * x[j] + a[i,q] * x[q] <= U[i]  ==>
*             j!=q
*
*     L[i] <= sum a[i,j] * x[j] + a[i,q] * (u[q] - x') <= U[i]  ==>
*             j!=q
*
*     L'[i] <= sum a[i,j] * x[j] - a[i,q] * x' <= U'[i],
*
*  where L'[i] = L[i] - a[i,q] * u[q], U'[i] = U[i] - a[i,q] * u[q]. */

static void rcv_ubnd_col(NPP *npp, void *info);

void npp_ubnd_col(NPP *npp, NPPCOL *col)
{     /* process column with upper bound */
      struct bnd_col *info;
      NPPROW *row;
      NPPAIJ *aij;
      NPPLFE *lfe;
      /* the column must have upper bound */
      xassert(col->ub != +DBL_MAX);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_ubnd_col, sizeof(struct bnd_col));
      info->q = col->j;
      info->b = col->ub;
      info->ptr = NULL;
      /* substitute x[q] into the objective function */
      npp->c0 += col->coef * col->ub;
      col->coef = - col->coef;
      /* substitute x[q] into rows and save the column coefficients */
      for (aij = col->ptr; aij != NULL; aij = aij->c_next)
      {  row = aij->row;
         if (row->lb == row->ub)
            row->ub = (row->lb -= aij->val * col->ub);
         else
         {  if (row->lb != -DBL_MAX)
               row->lb -= aij->val * col->ub;
            if (row->ub != +DBL_MAX)
               row->ub -= aij->val * col->ub;
         }
         lfe = dmp_get_atom(npp->stack, sizeof(NPPLFE));
         lfe->ref = row->i;
         lfe->val = aij->val, aij->val = - aij->val;
         lfe->next = info->ptr;
         info->ptr = lfe;
      }
      /* column x[q] becomes column x' */
      if (col->lb != -DBL_MAX)
         col->ub -= col->lb;
      else
         col->ub = +DBL_MAX;
      col->lb = 0.0;
      return;
}

static void rcv_ubnd_col(NPP *npp, void *_info)
{     /* recover column with upper bound */
      struct bnd_col *info = _info;
      NPPLFE *lfe;
      if (npp->sol == GLP_BS)
      {  if (npp->c_stat[info->q] == GLP_BS)
            /* x[q] remains basic */;
         else if (npp->c_stat[info->q] == GLP_NL)
            npp->c_stat[info->q] = GLP_NU;
         else if (npp->c_stat[info->q] == GLP_NU)
            npp->c_stat[info->q] = GLP_NL;
         else
            xassert(npp != npp);
      }
      /* compute variable x[q] = u[q] - x' */
      npp->c_prim[info->q] = info->b - npp->c_prim[info->q];
      /* compute auxiliary variables x[i] = x'[i] + a[i,q] * u[q] */
      for (lfe = info->ptr; lfe != NULL; lfe = lfe->next)
         npp->r_prim[lfe->ref] += lfe->val * info->b;
      /* determine reduced cost of x[q] */
      if (npp->sol != GLP_MIP)
         npp->c_dual[info->q] = - npp->c_dual[info->q];
      return;
}

/***********************************************************************
*  DOUBLE-BOUNDED COLUMN
*
*  Let column q be double-bounded with zero lower bound:
*
*     0 <= x[q] <= u[q].
*
*  Then its upper bound u[q] can be replaced by the following equality
*  constraint:
*
*     x[q] + x' = u[q],
*
*  where x' is a non-negative variable. */

struct dbnd_col
{     /* double-bounded column */
      int q;
      /* reference number of column x[q] */
      int j;
      /* reference number of column x' */
};

static void rcv_dbnd_col(NPP *npp, void *info);

void npp_dbnd_col(NPP *npp, NPPCOL *col)
{     /* process double-bounded column */
      struct dbnd_col *info;
      NPPROW *row;
      NPPCOL *ccc;
      /* the column must be double-bounded */
      xassert(col->lb == 0.0 && col->ub != +DBL_MAX);
      /* create variable x' */
      ccc = npp_add_col(npp);
      ccc->lb = 0.0, ccc->ub = +DBL_MAX;
      /* create equality constraint x[q] + x' = u[q] */
      row = npp_add_row(npp);
      row->lb = row->ub = col->ub;
      npp_add_aij(npp, row, col, +1.0);
      npp_add_aij(npp, row, ccc, +1.0);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_dbnd_col, sizeof(struct dbnd_col));
      info->q = col->j;
      info->j = ccc->j;
      /* remove upper bound of x[q] */
      col->ub = +DBL_MAX;
      return;
}

static void rcv_dbnd_col(NPP *npp, void *_info)
{     struct dbnd_col *info = _info;
      if (npp->sol == GLP_BS)
      {  /* note that x[q] and x' cannot be both non-basic */
         if (npp->c_stat[info->q] == GLP_NL)
            npp->c_stat[info->q] = GLP_NL;
         else if (npp->c_stat[info->j] == GLP_NL)
            npp->c_stat[info->q] = GLP_NU;
         else
            npp->c_stat[info->q] = GLP_BS;
      }
      /* variable x[q] is already computed */
      /* compute reduced cost of x[q] */
      if (npp->sol != GLP_MIP)
         npp->c_dual[info->q] -= npp->c_dual[info->j];
      return;
}

/***********************************************************************
*  FIXED COLUMN
*
*  Let column q be fixed:
*
*     x[q] = s[q].
*
*  where s[q] is a given value. Then it can be substituted and thereby
*  removed from the problem.
*
*  Substitution into the objective function gives:
*
*     z = sum c[j] * x[j] + c0 =
*          j
*
*       = sum c[j] * x[j] + c[q] * s[q] + c0 =
*         j!=q
*
*       = sum c[j] * x[j] + (c0)',
*         j!=q
*
*  where (c0)' = c0 + c[q] * s[q].
*
*  Substitution into i-th row, 1 <= i <= m, a[i,q] != 0, gives:
*
*     L[i] <= sum a[i,j] * x[j] <= U[i]  ==>
*              j
*
*     L[i] <= sum a[i,j] * x[j] + a[i,q] * s[q] <= U[i]  ==>
*             j!=q
*
*     L'[i] <= sum a[i,j] * x[j] + a[i,q] * x' <= U'[i],
*
*  where L'[i] = L[i] - a[i,q] * s[q], U'[i] = U[i] - a[i,q] * s[q].
*
*  Since x[q] is fixed, it is non-basic. Reduced cost of x[q] can be
*  computed using the dual equality constraint:
*
*     sum a[i,q] * pi[i] + lambda[q] = c[q],
*      i
*
*  from which it follows that:
*
*     lambda[q] = c[q] - sum a[i,q] * pi[i],
*                         i
*
*  where c[q] is objective coefficient at x[q], pi[i] are reduced costs
*  of corresponding auxiliary variables. */

struct fixed_col
{     /* fixed column */
      int q;
      /* reference number of the column */
      double s;
      /* value, at which the column is fixed */
      double c;
      /* objective coefficient */
      NPPLFE *ptr;
      /* list of non-zero coefficients a[i,q] */
};

static void rcv_fixed_col(NPP *npp, void *info);

void npp_fixed_col(NPP *npp, NPPCOL *col)
{     /* process fixed column */
      struct fixed_col *info;
      NPPROW *row;
      NPPAIJ *aij;
      NPPLFE *lfe;
      /* the column must be fixed */
      xassert(col->lb == col->ub);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_fixed_col, sizeof(struct fixed_col));
      info->q = col->j;
      info->s = col->lb;
      info->c = col->coef;
      info->ptr = NULL;
      /* substitute x[q] into the objective function */
      npp->c0 += col->coef * col->lb;
      /* substitute x[q] into rows and save the column coefficients */
      for (aij = col->ptr; aij != NULL; aij = aij->c_next)
      {  row = aij->row;
         if (row->lb == row->ub)
            row->ub = (row->lb -= aij->val * col->lb);
         else
         {  if (row->lb != -DBL_MAX)
               row->lb -= aij->val * col->lb;
            if (row->ub != +DBL_MAX)
               row->ub -= aij->val * col->lb;
         }
         lfe = dmp_get_atom(npp->stack, sizeof(NPPLFE));
         lfe->ref = aij->row->i;
         lfe->val = aij->val;
         lfe->next = info->ptr;
         info->ptr = lfe;
      }
      /* remove x[q] from the problem */
      npp_del_col(npp, col);
      return;
}

static void rcv_fixed_col(NPP *npp, void *_info)
{     /* recover fixed column */
      struct fixed_col *info = _info;
      NPPLFE *lfe;
      double sum;
      if (npp->sol == GLP_SOL)
      {  /* the fixed column is non-basic */
         npp->c_stat[info->q] = GLP_NS;
      }
      /* determine variable x[q] = s[q] */
      npp->c_prim[info->q] = info->s;
      /* compute auxiliary variables x[i] = x'[i] + a[i,q] * s[q] */
      for (lfe = info->ptr; lfe != NULL; lfe = lfe->next)
         npp->r_prim[lfe->ref] += lfe->val * info->s;
      /* compute reduced cost of x[q] */
      if (npp->sol != GLP_MIP)
      {  sum = info->c;
         for (lfe = info->ptr; lfe != NULL; lfe = lfe->next)
            sum -= lfe->val * npp->r_dual[lfe->ref];
         npp->c_dual[info->q] = sum;
      }
      return;
}

/***********************************************************************
*  EMPTY ROW
*
*  Let row p be empty:
*
*     L[p] <= sum 0 * x[j] <= U[p].
*
*  If L[p] <= 0 <= U[p], then the row is redundant and can be removed
*  from the problem. Otherwise, the row is primal infeasible. */

struct empty_row
{     /* empty row */
      int p;
      /* reference number of the row */
};

static void rcv_empty_row(NPP *npp, void *info);

int npp_empty_row(NPP *npp, NPPROW *row)
{     /* process empty row */
      struct empty_row *info;
      double eps = 1e-3;
      /* the row must be empty */
      xassert(row->ptr == NULL);
      /* check primal feasibility */
      if (row->lb > +eps || row->ub < -eps)
         return 1;
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_empty_row, sizeof(struct empty_row));
      info->p = row->i;
      /* remove the row from the problem */
      npp_del_row(npp, row);
      return 0;
}

static void rcv_empty_row(NPP *npp, void *_info)
{     /* recover empty row */
      struct empty_row *info = _info;
      if (npp->sol == GLP_SOL)
      {  /* the empty row is non-active */
         npp->r_stat[info->p] = GLP_BS;
      }
      /* auxiliary variable x[p] is zero */
      npp->r_prim[info->p] = 0.0;
      /* and its reduced cost is also zero */
      if (npp->sol != GLP_MIP)
         npp->r_dual[info->p] = 0.0;
      return;
}

/***********************************************************************
*  EMPTY COLUMN
*
*  Let column q be empty, i.e. have zero coefficients in all rows.
*
*  If c[q] = 0, x[q] can be fixed at any feasible value. If c[q] < 0,
*  x[q] must be fixed at its lower bound, and if c[q] > 0, x[q] must be
*  fixed at its upper bound. If x[q] has no appropriate lower/upper
*  bound to be fixed at, the column is dual infeasible. */

struct empty_col
{     /* empty column */
      int q;
      /* reference number of the column */
      int stat;
      /* status in basic solution */
};

static void rcv_empty_col(NPP *npp, void *info);

int npp_empty_col(NPP *npp, NPPCOL *col)
{     /* process empty column */
      struct empty_col *info;
      double eps = 1e-3;
      /* the column must be empty */
      xassert(col->ptr == NULL);
      /* check dual feasibility */
      if (col->coef > +eps && col->lb == -DBL_MAX)
         return 1;
      if (col->coef < -eps && col->ub == +DBL_MAX)
         return 1;
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_empty_col, sizeof(struct empty_col));
      info->q = col->j;
      /* fix the column */
      if (col->lb == -DBL_MAX && col->ub == +DBL_MAX)
      {  /* free variable */
         info->stat = GLP_NS;
         col->lb = col->ub = 0.0;
      }
      else if (col->ub == +DBL_MAX)
lo:   {  /* variable with lower bound */
         info->stat = GLP_NL;
         col->ub = col->lb;
      }
      else if (col->lb == -DBL_MAX)
up:   {  /* variable with upper bound */
         info->stat = GLP_NU;
         col->lb = col->ub;
      }
      else if (col->lb != col->ub)
      {  /* double-bounded variable */
         if (col->coef > 0.0) goto lo;
         if (col->coef < 0.0) goto up;
         if (fabs(col->lb) <= fabs(col->ub)) goto lo; else goto up;
      }
      else
      {  /* fixed variable */
         info->stat = GLP_NS;
      }
      /* process fixed column */
      npp_fixed_col(npp, col);
      return 0;
}

static void rcv_empty_col(NPP *npp, void *_info)
{     /* recover empty column */
      struct empty_col *info = _info;
      if (npp->sol == GLP_SOL)
         npp->c_stat[info->q] = (char)info->stat;
      return;
}

/***********************************************************************
*  npp_implied_fixed - process implied fixed value of column
*
*  This routine processes an implied fixed value of the specified
*  non-fixed column and returns one of the following codes:
*
*  0 - column has been fixed at the implied value;
*  1 - primal infeasibility detected;
*  2 - integer infeasibility detected. */

int npp_implied_fixed(NPP *npp, NPPCOL *col, double val)
{     int ret;
      double eps, temp;
      /* column must not be fixed */
      xassert(col->lb != col->ub);
      /* check integer feasibility */
      if (npp->sol == GLP_MIP && col->kind == GLP_IV)
      {  temp = floor(val + 0.5);
         if (fabs(val - temp) <= 1e-5)
            val = temp;
         else
         {  ret = 2;
            goto done;
         }
      }
      /* check primal feasibility for current lower bound */
      if (col->lb != -DBL_MAX)
      {  eps = 1e-3 + 1e-6 * fabs(col->lb);
         if (val < col->lb - eps)
         {  ret = 1;
            goto done;
         }
      }
      /* check primal feasibility for current upper bound */
      if (col->ub != +DBL_MAX)
      {  eps = 1e-3 + 1e-6 * fabs(col->ub);
         if (val > col->ub + eps)
         {  ret = 1;
            goto done;
         }
      }
      /* fix column at the implied value */
      col->lb = col->ub = val;
      ret = 0;
done: return ret;
}

/***********************************************************************
*  ROW SINGLETON (EQUALITY CONSTRAINT)
*
*  Let row p be an equality constraint having the only column:
*
*     a[p,q] * x[q] = b[p].
*
*  Then it implies fixing x[q]:
*
*     x[q] = b[p] / a[p,q].
*
*  If this implied value of x[q] does not conflict with its bounds, it
*  can be fixed, in which case row p becomes redundant. Otherwise, the
*  row is primal infeasible.
*
*  On entry to the recovering routine the column q is already recovered
*  as if it were a fixed column, i.e. it is non-basic with primal value
*  x[q] and reduced cost lambda[q]. The routine makes it basic with the
*  same primal value and zero reduced cost.
*
*  Then the recovering routine makes the row p non-basic, and computes
*  its primal value x[p] = a[p,q] * x[q] as well as its reduced cost:
*
*     lambda[p] = lambda[q] / a[p,q],
*
*  where lambda[q] is a dual value, which the column q has on entry to
*  the recovering routine.
*
*  RETURNS
*
*  0 - operation successful;
*  1 - primal infeasibility detected;
*  2 - integer infeasibility detected. */

struct row_sngtn1
{     /* row singleton (equality constraint) */
      int p;
      /* reference number of the row */
      int q;
      /* reference number of the column */
      double apq;
      /* constraint coefficient */
};

static void rcv_row_sngtn1(NPP *npp, void *info);

int npp_row_sngtn1(NPP *npp, NPPROW *row)
{     /* process row singleton (equality constraint) */
      struct row_sngtn1 *info;
      NPPCOL *col;
      NPPAIJ *aij;
      int ret;
      double val;
      /* the row must be singleton equality constraint */
      xassert(row->lb == row->ub);
      xassert(row->ptr != NULL && row->ptr->r_next == NULL);
      /* compute and process implied fixed value of x[q] */
      aij = row->ptr;
      col = aij->col;
      val = row->lb / aij->val;
      ret = npp_implied_fixed(npp, col, val);
      if (ret != 0) goto done;
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_row_sngtn1, sizeof(struct row_sngtn1));
      info->p = row->i;
      info->q = col->j;
      info->apq = aij->val;
      /* remove the row from the problem */
      npp_del_row(npp, row);
      ret = 0;
done: return ret;
}

static void rcv_row_sngtn1(NPP *npp, void *_info)
{     /* recover row singleton (equality constraint) */
      struct row_sngtn1 *info = _info;
      /* x[q] is already recovered */
      if (npp->sol == GLP_SOL)
      {  xassert(npp->c_stat[info->q] == GLP_NS);
         npp->r_stat[info->p] = GLP_NS;
         npp->c_stat[info->q] = GLP_BS;
      }
      /* compute auxiliary variable x[p] */
      npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
      /* compute reduced cost of x[p] and x[q] */
      if (npp->sol != GLP_MIP)
      {  npp->r_dual[info->p] = npp->c_dual[info->q] / info->apq;
         npp->c_dual[info->q] = 0.0;
      }
      return;
}

/***********************************************************************
*  npp_implied_lower - process implied lower bound of column
*
*  This routine processes an implied lower bound of the specified
*  non-fixed column and returns one of the following codes:
*
*  0 - bound not changed, since it is not weaker than implied one;
*  1 - bound changed, but not significantly;
*  2 - bound changed significantly;
*  3 - column has been fixed due to bound change;
*  4 - primal infeasibility detected. */

int npp_implied_lower(NPP *npp, NPPCOL *col, double bnd)
{     int ret;
      double eps, temp;
      /* column must not be fixed */
      xassert(col->lb != col->ub);
      /* the implied lower bound must be finite */
      xassert(bnd != -DBL_MAX);
      /* round the implied lower bound of integer-valued column */
      if (npp->sol == GLP_MIP && col->kind == GLP_IV)
      {  temp = floor(bnd + 0.5);
         if (fabs(bnd - temp) <= 1e-5)
            bnd = temp;
         else
            bnd = ceil(bnd);
      }
      /* check if current lower bound is weaker */
      if (col->lb != -DBL_MAX)
      {  eps = 1e-6 + 1e-9 * fabs(col->lb);
         if (bnd < col->lb + eps)
         {  ret = 0; /* not weaker */
            goto done;
         }
      }
      /* check primal feasibility for current upper bound */
      if (col->ub != +DBL_MAX)
      {  eps = 1e-3 + 1e-6 * fabs(col->ub);
         if (bnd > col->ub + eps)
         {  ret = 4; /* infeasible */
            goto done;
         }
      }
      /* check if bound changes significantly */
      if (col->lb != -DBL_MAX
            && (bnd - col->lb) / (1.0 + fabs(bnd)) < 0.20)
         ret = 1;
      else
         ret = 2;
      /* set new lower bound */
      col->lb = bnd;
      /* fix column, if necessary */
      if (col->ub != +DBL_MAX)
      {  eps = 1e-9 + 1e-12 * fabs(col->ub);
         if (bnd > col->ub - eps)
         {  col->lb = col->ub;
            ret = 3;
         }
      }
done: return ret;
}

/***********************************************************************
*  npp_implied_upper - process implied upper bound of column
*
*  This routine processes an implied upper bound of the specified
*  non-fixed column and returns one of the following codes:
*
*  0 - bound not changed, since it is not weaker than implied one;
*  1 - bound changed, but not significantly;
*  2 - bound changed significantly;
*  3 - column has been fixed due to bound change;
*  4 - primal infeasibility detected. */

int npp_implied_upper(NPP *npp, NPPCOL *col, double bnd)
{     int ret;
      double eps, temp;
      /* column must not be fixed */
      xassert(col->lb != col->ub);
      /* the implied upper bound must be finite */
      xassert(bnd != +DBL_MAX);
      /* round the implied upper bound of integer-valued column */
      if (npp->sol == GLP_MIP && col->kind == GLP_IV)
      {  temp = floor(bnd + 0.5);
         if (fabs(bnd - temp) <= 1e-5)
            bnd = temp;
         else
            bnd = floor(bnd);
      }
      /* check if current upper bound is weaker */
      if (col->ub != +DBL_MAX)
      {  eps = 1e-6 + 1e-9 * fabs(col->ub);
         if (bnd > col->ub - eps)
         {  ret = 0; /* not weaker */
            goto done;
         }
      }
      /* check primal feasibility for current lower bound */
      if (col->lb != -DBL_MAX)
      {  eps = 1e-3 + 1e-6 * fabs(col->lb);
         if (bnd < col->lb - eps)
         {  ret = 4; /* infeasible */
            goto done;
         }
      }
      /* check if bound changes significantly */
      if (col->ub != +DBL_MAX
            && (col->ub - bnd) / (1.0 + fabs(bnd)) < 0.20)
         ret = 1;
      else
         ret = 2;
      /* set new upper bound */
      col->ub = bnd;
      /* fix column, if necessary */
      if (col->lb != -DBL_MAX)
      {  eps = 1e-9 + 1e-12 * fabs(col->lb);
         if (bnd < col->lb + eps)
         {  col->ub = col->lb;
            ret = 3;
         }
      }
done: return ret;
}

/***********************************************************************
*  ROW SINGLETON (INEQUALITY CONSTRAINT)
*
*  Let row p be an inequality constraint having the only column:
*
*     l[p] <= a[p,q] * x[q] <= u[p].                                 (1)
*
*  where l[p] != u[p] and at least one of the bounds l[p] and u[p] is
*  finite. Then the row implies bounds of x[q]:
*
*     l' <= x[q] <= u',                                              (2)
*
*  where:
*
*     if a[p,q] > 0:   l' = l[p] / a[p,q],   u' = u[p] / a[p,q];     (3)
*
*     if a[p,q] < 0:   l' = u[p] / a[p,q],   u' = l[p] / a[p,q].     (4)
*
*  If the bounds l' and u' conflict with own bounds of x[q], i.e. if
*  l' > u[q] or u' < l[q], the row is primal infeasible. Otherwise the
*  range of x[q] can be tightened as follows:
*
*     max(l[q], l') <= x[q] <= min(u[q], u'),                        (5)
*
*  in which case the row becomes redundant and therefore can be removed
*  from the problem.
*
*  On entry to the recovering routine the column q is already recovered
*  as if it had the modified bounds (5). Then the row p is recovered as
*  follows.
*
*  Let the column q be basic. In this case the row p is inactive, so
*  its primal value x[p] is computed with the formula (1) and its dual
*  value pi[p] is set to zero.
*
*  Let the column q be non-basic on lower bound. If l' <= l[q], the row
*  p is inactive and recovered in the same way as above. Otherwise, if
*  l' > l[q], the column q is actually basic while the row p is active
*  on its lower bound (a[p,q] > 0) or on its upper bound (a[p,q] < 0).
*  In the latter case the row's primal value y[p] is computed using the
*  formula (1) and the dual value pi[p] is computed using the formula:
*
*     pi[p] = lambda[q] / a[p,q],                                    (6)
*
*  where lambda[q] is a dual value, which the column q has on entry to
*  the recovering routine.
*
*  Let the column q be non-basic on upper bound. If u' >= u[q], the row
*  p is inactive and recovered in the same way as above. Otherwise, if
*  u' < u[q], the column q is actually basic while the row p is active
*  on its lower bound (a[p,q] < 0) or on its upper bound (a[p,q] > 0).
*  Then the row's primal value x[p] is computed using the formula (1)
*  and the dual value pi[p] is computed using the formula (6).
*
*  RETURNS
*
*  0 - bounds not changed
*  1 - bounds changed, but not significantly;
*  2 - bounds changed significantly;
*  3 - column has been fixed due to bounds change (note that the column
*      is not removed by the routine);
*  4 - primal infeasibility detected. */

struct row_sngtn2
{     /* row singleton (inequality constraint) */
      int p;
      /* reference number of the row */
      int q;
      /* reference number of the column */
      double apq;
      /* constraint coefficient */
      char lb_changed;
      /* this flag is set if the lower bound of the column was changed,
         i.e. if l' > l[q]; see (2)-(5) */
      char ub_changed;
      /* this flag is set if the upper bound of the column was changed,
         i.e. if u' < u[q]; see (2)-(5) */
};

static void rcv_row_sngtn2(NPP *npp, void *info);

int npp_row_sngtn2(NPP *npp, NPPROW *row)
{     /* process row singleton (inequality constraint) */
      struct row_sngtn2 *info;
      NPPCOL *col;
      NPPAIJ *aij;
      int ret;
      double lb, ub;
      char lb_changed, ub_changed;
      /* the row must be singleton inequality constraint */
      xassert(row->lb != -DBL_MAX || row->ub != +DBL_MAX);
      xassert(row->lb != row->ub);
      xassert(row->ptr != NULL && row->ptr->r_next == NULL);
      /* compute implied bounds l' and u'; see (2)-(4) */
      aij = row->ptr;
      col = aij->col;
      xassert(col->lb != col->ub);
      if (aij->val > 0.0)
      {  lb = (row->lb == -DBL_MAX ? -DBL_MAX : row->lb / aij->val);
         ub = (row->ub == +DBL_MAX ? +DBL_MAX : row->ub / aij->val);
      }
      else
      {  lb = (row->ub == +DBL_MAX ? -DBL_MAX : row->ub / aij->val);
         ub = (row->lb == -DBL_MAX ? +DBL_MAX : row->lb / aij->val);
      }
      /* process implied lower bound */
      if (lb == -DBL_MAX)
      {  ret = 0;
         lb_changed = 0;
      }
      else
      {  switch (npp_implied_lower(npp, col, lb))
         {  case 0:
               /* lower bound not changed */
               ret = 0;
               lb_changed = 0;
               break;
            case 1:
               /* lower bound changed, but not significantly */
               ret = 1;
               lb_changed = 1;
               break;
            case 2:
               /* lower bound changed significantly */
               ret = 2;
               lb_changed = 1;
               break;
            case 3:
               /* column has been fixed */
               ret = 3;
               lb_changed = 1;
               /* skip processing implied upper bound (may note that
                  l' <= u') */
               ub = +DBL_MAX;
               break;
            case 4:
               /* primal infeasibility detected */
               ret = 4;
               goto done;
            default:
               xassert(ret != ret);
         }
      }
      /* process implied upper bound */
      if (ub == +DBL_MAX)
         ub_changed = 0;
      else
      {  switch (npp_implied_upper(npp, col, ub))
         {  case 0:
               /* upper bound not changed */
               ub_changed = 0;
               break;
            case 1:
               /* upper bound changed, but not significantly */
               if (ret < 1) ret = 1;
               ub_changed = 1;
               break;
            case 2:
               /* upper bound changed significantly */
               if (ret < 2) ret = 2;
               ub_changed = 1;
               break;
            case 3:
               /* column has been fixed */
               if (ret < 3) ret = 3;
               ub_changed = 1;
               break;
            case 4:
               /* primal infeasibility detected */
               ret = 4;
               goto done;
            default:
               xassert(ret != ret);
         }
      }
      /* if neither lower nor upper bound of the column were changed,
         the row is originally redundant and can be made free */
      if (ret == 0)
      {  xassert(!lb_changed && !ub_changed);
         row->lb = -DBL_MAX, row->ub = +DBL_MAX;
         npp_free_row(npp, row);
         goto done;
      }
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_row_sngtn2, sizeof(struct row_sngtn2));
      info->p = row->i;
      info->q = col->j;
      info->apq = aij->val;
      info->lb_changed = lb_changed;
      info->ub_changed = ub_changed;
      /* remove the row from the problem */
      npp_del_row(npp, row);
done: return ret;
}

static void rcv_row_sngtn2(NPP *npp, void *_info)
{     /* recover row singleton (inequality constraint) */
      struct row_sngtn2 *info = _info;
      /* the column is already recovered while the row is not recovered
         yet */
      if (npp->sol == GLP_SOL)
      {  /* recover basic solution */
         if (npp->c_stat[info->q] == GLP_BS)
         {  /* the column is basic, so the row is inactive */
            npp->r_stat[info->p] = GLP_BS;
            npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
            npp->r_dual[info->p] = 0.0;
         }
         else if (npp->c_stat[info->q] == GLP_NL)
nl:      {  /* the column is non-basic on its lower bound */
            if (info->lb_changed)
            {  /* it is not its own lower bound, so actually the row is
                  active */
               if (info->apq > 0.0)
                  npp->r_stat[info->p] = GLP_NL;
               else
                  npp->r_stat[info->p] = GLP_NU;
               npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
               npp->r_dual[info->p] = npp->c_dual[info->q] / info->apq;
               /* and the column is basic */
               npp->c_stat[info->q] = GLP_BS;
               npp->c_dual[info->q] = 0.0;
            }
            else
            {  /* it is its own lower bound, so the row is inactive */
               npp->r_stat[info->p] = GLP_BS;
               npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
               npp->r_dual[info->p] = 0.0;
            }
         }
         else if (npp->c_stat[info->q] == GLP_NU)
nu:      {  /* the column is non-basic on its upper bound */
            if (info->ub_changed)
            {  /* it is not its own upper bound, so actually the row is
                  active */
               if (info->apq > 0.0)
                  npp->r_stat[info->p] = GLP_NU;
               else
                  npp->r_stat[info->p] = GLP_NL;
               npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
               npp->r_dual[info->p] = npp->c_dual[info->q] / info->apq;
               /* and the column is basic */
               npp->c_stat[info->q] = GLP_BS;
               npp->c_dual[info->q] = 0.0;
            }
            else
            {  /* it is its own upper bound, so the row is inactive */
               npp->r_stat[info->p] = GLP_BS;
               npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
               npp->r_dual[info->p] = 0.0;
            }
         }
         else if (npp->c_stat[info->q] == GLP_NF)
         {  /* the column cannot be free, since the row is always has
               at least one finite bound; see (5) */
            xassert(npp != npp);
         }
         else if (npp->c_stat[info->q] == GLP_NS)
         {  /* the column is non-basic and fixed; it cannot be fixed
               before tightening its bounds, because in that case this
               transformation entry is not created; therefore we need
               to consider the column as double-bounded whose lower and
               upper bounds are equal to each other, and choose a bound
               using dual value lambda[q] */
            if (npp->c_dual[info->q] >= 0.0)
            {  /* lambda[q] >= 0; set the column on lower bound */
               npp->c_stat[info->q] = GLP_NL;
               goto nl;
            }
            else
            {  /* lambda[q] < 0; set the column on upper bound */
               npp->c_stat[info->q] = GLP_NU;
               goto nu;
            }
         }
         else
            xassert(npp != npp);
      }
      else if (npp->sol == GLP_IPT)
      {  /* recover interior-point solution */
         if (npp->c_dual[info->q] > 0.0 && info->lb_changed ||
             npp->c_dual[info->q] < 0.0 && info->ub_changed)
         {  /* the column has active bound, which is not its own bound,
               so actually the row is active */
            npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
            npp->r_dual[info->p] = npp->c_dual[info->q] / info->apq;
            npp->c_dual[info->q] = 0.0;
         }
         else
         {  /* either the column has no active bound (lambda[q] = 0),
               or the active bound is its own bound; in both cases the
               row is inactive */
            npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
            npp->r_dual[info->p] = 0.0;
         }
      }
      else if (npp->sol == GLP_MIP)
      {  /* recover MIP solution */
         npp->r_prim[info->p] = info->apq * npp->c_prim[info->q];
      }
      else
         xassert(npp != npp);
      return;
}

/***********************************************************************
*  COLUMN SINGLETON (IMPLIED SLACK VARIABLE)
*
*  Let column q have the only non-zero constraint coefficient in row p,
*  which is an equality constraint:
*
*     x[p] =  sum a[p,j] * x[j] + a[p,q] * x[q] = b[p],              (1)
*            j!=q
*
*     l[q] <= x[q] <= u[q].                                          (2)
*
*  The term a[p,q] * x[q] can be considered as a slack variable of the
*  row p, that allows removing the column q from the problem.
*
*  From (1) it follows that:
*
*     sum a[p,j] * x[j] = b[p] - a[p,q] * x[q].                      (3)
*    j!=q
*
*  So, (1) can be replaced by the following equivalent constraint:
*
*     l' <= x' =  sum a[p,j] * x[j] <= u',                           (4)
*                j!=q
*
*  where x' is an auxiliary variable of this new constraint, and
*
*     if a[p,q] > 0:   l' = b[p] - a[p,q] * u[q],                    (5)
*                      u' = b[p] - a[p,q] * l[q],
*
*     if a[p,q] < 0:   l' = b[p] - a[p,q] * l[q],                    (6)
*                      u' = b[p] - a[p,q] * u[q].
*
*  On removing x[q] from the problem it also should be substituted in
*  the objective function. From (3) it follows that:
*
*     x[q] = - sum (a[p,j] / a[p,q]) * x[j] + b[p] / a[p,q],         (7)
*             j!=q
*
*  and substituting x[q] in the objective function gives:
*
*     z = sum c[j] * x[j] + c[0] =
*          j
*
*       = sum c[j] * x[j] + c[q] * x[q] + c[0] =                     (8)
*        j!=q
*
*       = sum c'[j] * x[j] + c'[0],
*        j!=q
*
*  where:
*
*     c'[j] = c[j] - c[q] * (a[p,j] / a[p,q]),                       (9)
*
*     c'[0] = c[0] + c[q] * (b[p] / a[p,q]).                        (10)
*
*  On entry to the recovering routine the row (4) that corresponds to
*  the row p is already recovered, i.e. its status, primal value x' and
*  dual value pi' are known.
*
*  From (3) and (4) it follows that
*
*     x' = b[p] - a[p,q] * x[q].                                    (11)
*
*  Therefore the status of the column q is determined as follows:
*
*  if x' is basic, x[q] is basic;
*
*  if x' is non-basic on lower bound, x[q] is non-basic on upper (if
*  a[p,q] > 0) or lower (if a[p,q] < 0) bound;
*
*  if x' is non-basic on upper bound, x[q] is non-basic on lower (if
*  a[p,q] > 0) or upper (of a[p,q] < 0) bound.
*
*  The primal and dual values of the column q are computed as follows:
*
*     x[q] = (b[p] - x') / a[p,q],                                  (12)
*
*     lambda[q] = - a[p,q] * pi',                                   (13)
*
*  where x' and pi' are primal and dual values of the row (4), resp.
*
*  Being an equality constraint the row p is active.
*
*  From (1) it follows that
*
*     x[p] = x' + a[p,q] * x[q] = b[p],                             (14)
*
*  which is the formula to compute the primal value of the row p.
*
*  The dual equality constraint for the row p is:
*
*     a[p,q] * pi[p] + lambda[q] = c[q],                            (15)
*
*  therefore
*
*     pi[p] = (c[q] - lambda[q]) / a[p,q],                          (16)
*
*  which is the formula to compute the dual value of the row p. */

struct col_sngtn1
{     /* column singleton (implied slack variable) */
      int p;
      /* reference number of the row */
      int q;
      /* reference number of the column */
      double rhs;
      /* b[p], right-hand side of the row */
      double coef;
      /* c[q], objective coefficient of the column */
      double apq;
      /* a[p,q], constraint coefficient */
};

static void rcv_col_sngtn1(NPP *npp, void *info);

void npp_col_sngtn1(NPP *npp, NPPCOL *col)
{     /* process column singleton (implied slack variable) */
      struct col_sngtn1 *info;
      NPPROW *row;
      NPPAIJ *aij;
      double lb, ub;
      /* the column must be non-integral non-fixed singleton */
      if (npp->sol == GLP_MIP) xassert(col->kind != GLP_IV);
      xassert(col->lb != col->ub);
      xassert(col->ptr != NULL && col->ptr->c_next == NULL);
      /* the corresponding row must be equality constraint */
      aij = col->ptr;
      row = aij->row;
      xassert(row->lb == row->ub);
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_col_sngtn1, sizeof(struct col_sngtn1));
      info->p = row->i;
      info->q = col->j;
      info->rhs = row->lb;
      info->coef = col->coef;
      info->apq = aij->val;
      /* compute new bounds l' and u' of the row; see (4)-(6) */
      if (info->apq > 0.0)
      {  lb = (col->ub ==
            +DBL_MAX ? -DBL_MAX : info->rhs - info->apq * col->ub);
         ub = (col->lb ==
            -DBL_MAX ? +DBL_MAX : info->rhs - info->apq * col->lb);
      }
      else
      {  lb = (col->lb ==
            -DBL_MAX ? -DBL_MAX : info->rhs - info->apq * col->lb);
         ub = (col->ub ==
            +DBL_MAX ? +DBL_MAX : info->rhs - info->apq * col->ub);
      }
      row->lb = lb;
      row->ub = ub;
      /* remove the column from the problem */
      npp_del_col(npp, col);
      /* substitute x[q] into the objective function; see (7)-(10) */
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
         aij->col->coef -= info->coef * (aij->val / info->apq);
      npp->c0 += info->coef * (info->rhs / info->apq);
      return;
}

static void rcv_col_sngtn1(NPP *npp, void *_info)
{     /* recover column singleton (implied slack variable) */
      struct col_sngtn1 *info = _info;
      /* the (modified) row is already recovered while the column is
         not recovered yet */
      if (npp->sol == GLP_SOL)
      {  /* recover basic solution */
         if (npp->r_stat[info->p] == GLP_BS)
         {  /* x' is basic */
            npp->c_stat[info->q] = GLP_BS;
         }
         else if (npp->r_stat[info->p] == GLP_NL)
nl:      {  /* x' is non-basic on its lower bound */
            npp->c_stat[info->q] =
               (char)(info->apq > 0.0 ? GLP_NU : GLP_NL);
         }
         else if (npp->r_stat[info->p] == GLP_NU)
nu:      {  /* x' is non-basic on its upper bound */
            npp->c_stat[info->q] =
               (char)(info->apq > 0.0 ? GLP_NL : GLP_NU);
         }
         else if (npp->r_stat[info->p] == GLP_NF)
         {  /* x' is non-basic free */
            npp->c_stat[info->q] = GLP_NF;
         }
         else if (npp->r_stat[info->p] == GLP_NS)
         {  /* x' is non-basic fixed; this can happen if the row was
               turned to equality constraint, because if the column is
               of fixed type, this transformation entry is not created;
               thus, we need to consider the row as ranged, whose lower
               and upper bounds are equal to each other, and choose an
               appropriate bound using the dual value pi[p] */
            if (npp->r_dual[info->p] >= 0.0) goto nl; else goto nu;
         }
         else
            xassert(npp != npp);
         /* recover primal value x[q] using the formula (12) */
         npp->c_prim[info->q] =
            (info->rhs - npp->r_prim[info->p]) / info->apq;
         /* recover dual value lambda[q] using the formula (13) */
         npp->c_dual[info->q] = - info->apq * npp->r_dual[info->p];
         /* the row is active equality constraint */
         npp->r_stat[info->p] = GLP_NS;
         /* recover primal value x[p] using the formula (14) */
         npp->r_prim[info->p] = info->rhs;
         /* recover dual value pi[p] using the formula (15) */
         npp->r_dual[info->p] =
            (info->coef - npp->c_dual[info->q]) / info->apq;
      }
      else if (npp->sol == GLP_IPT)
      {  /* recover interior-point solution */
         /* recover primal value x[q] using the formula (12) */
         npp->c_prim[info->q] =
            (info->rhs - npp->r_prim[info->p]) / info->apq;
         /* recover dual value lambda[q] using the formula (13) */
         npp->c_dual[info->q] = - info->apq * npp->r_dual[info->p];
         /* recover primal value x[p] using the formula (14) */
         npp->r_prim[info->p] = info->rhs;
         /* recover dual value pi[p] using the formula (15) */
         npp->r_dual[info->p] =
            (info->coef - npp->c_dual[info->q]) / info->apq;
      }
      else
      {  /* recover MIP solution */
         /* recover primal value x[q] using the formula (12) */
         npp->c_prim[info->q] =
            (info->rhs - npp->r_prim[info->p]) / info->apq;
         /* recover primal value x[p] using the formula (14) */
         npp->r_prim[info->p] = info->rhs;
      }
      return;
}

/***********************************************************************
*  COLUMN SINGLETON (IMPLIED FREE VARIABLE)
*
*  Let column q have the only non-zero constraint coefficient in row p,
*  which is an inequality constraint:
*
*     l[p] <= x[p] =  sum a[p,j] * x[j] + a[p,q] * x[q] <= u[p],     (1)
*                    j!=q
*
*     l[q] <= x[q] <= u[q].                                          (2)
*
*  Resolving (1) through x[q] we have:
*
*     x[q] = a'[p,q] * x[p] + sum a'[p,j] * x[j],                    (3)
*                            j!=q
*
*  where
*
*     a'[p,q] = 1 / a[p,q],                                          (4)
*
*     a'[p,j] = - a[p,j] / a[p,q],                                   (5)
*
*  The formula (3) allows computing implied lower and upper bounds of
*  x[q] using explicit bounds of x[p] and x[j]. Let these impled bounds
*  of x[q] be l' and u'. If l' >= l[q] and u' <= u[q], own bounds of
*  x[q] are never reached, in which case x[q] can be considered as an
*  implied free variable.
*
*  Let x[q] be a free variable. Then it is always basic, and its dual
*  value lambda[q] is zero. If x[q] is basic, x[p] must be non-basic
*  (otherwise there were two linearly dependent columns in the basic
*  matrix). The dual value pi[p] can be found from the dual equality
*  constraint for the column q:
*
*     a[p,q] * pi[p] + lambda[q] = c[q],                             (6)
*
*  that gives:
*
*     pi[p] = (c[q] - lambda[q]) / a[p,q] = c[q] / a[p,q].           (7)
*
*  If pi[p] > 0, the row p must be active on its lower bound l[p], and
*  if the row p has no lower bound, it is dual infeasible. Analogously,
*  if pi[p] < 0, the row p must be active on its upper bound u[p], and
*  if the row p has no upper bound, it is dual infeasible. Thus, in both
*  cases the row p becomes an equality constraint, whose right-hand side
*  is l[p] or u[p] depending on the sign of pi[p], while the column q
*  becomes an implied slack variable. If pi[p] = 0 (i.e. if c[q] = 0),
*  the row p can be fixed at any bound.
*
*  The only thing needed on recovering is to properly set the status of
*  the row p, because on entry to the recovering routine this row is an
*  active equality constraint while actually it is an active inequality
*  constraint.
*
*  RETURNS
*
*  0 - operation successful;
*  1 - column may have active lower/upper bound, thus not removed;
*  2 - dual infeasibility detected. */

struct col_sngtn2
{     /* column singleton (implied free variable) */
      int p;
      /* row reference number */
      int q;
      /* column reference number */
      char stat;
      /* row status:
         GLP_NL - active constraint on lower bound
         GLP_NU - active constraint on upper bound */
};

static void rcv_col_sngtn2(NPP *npp, void *info);

int npp_col_sngtn2(NPP *npp, NPPCOL *col)
{     /* process column singleton (implied free variable) */
      struct col_sngtn2 *info;
      NPPROW *row;
      NPPAIJ *apq, *aij;
      double lb, ub, eps, temp, ret;
      /* the column must be non-integral non-fixed singleton */
      if (npp->sol == GLP_MIP) xassert(col->kind != GLP_IV);
      xassert(col->lb != col->ub);
      xassert(col->ptr != NULL && col->ptr->c_next == NULL);
      /* the corresponding row must be inequality constraint */
      apq = col->ptr;
      row = apq->row;
      xassert(row->lb != -DBL_MAX || row->ub != +DBL_MAX);
      xassert(row->lb != row->ub);
      /* compute l', an implied lower bound of x[q]; see (3)-(5) */
      temp = 1.0 / apq->val;
      if (temp > 0.0)
         lb = (row->lb == -DBL_MAX ? -DBL_MAX : temp * row->lb);
      else
         lb = (row->ub == +DBL_MAX ? -DBL_MAX : temp * row->ub);
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  if (lb == -DBL_MAX) break;
         if (aij == apq) continue;
         temp = - aij->val / apq->val;
         if (temp > 0.0)
         {  if (aij->col->lb == -DBL_MAX)
               lb = -DBL_MAX;
            else
               lb += temp * aij->col->lb;
         }
         else
         {  if (aij->col->ub == +DBL_MAX)
               lb = -DBL_MAX;
            else
               lb += temp * aij->col->ub;
         }
      }
      /* compute u', an implied upper bound of x[q]; see (3)-(5) */
      temp = 1.0 / apq->val;
      if (temp > 0.0)
         ub = (row->ub == +DBL_MAX ? +DBL_MAX : temp * row->ub);
      else
         ub = (row->lb == -DBL_MAX ? +DBL_MAX : temp * row->lb);
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  if (ub == +DBL_MAX) break;
         if (aij == apq) continue;
         temp = - aij->val / apq->val;
         if (temp > 0.0)
         {  if (aij->col->ub == +DBL_MAX)
               ub = +DBL_MAX;
            else
               ub += temp * aij->col->ub;
         }
         else
         {  if (aij->col->lb == -DBL_MAX)
               ub = +DBL_MAX;
            else
               ub += temp * aij->col->lb;
         }
      }
      /* check if x[q] can reach its own lower bound */
      if (col->lb != -DBL_MAX)
      {  eps = 1e-9 + 1e-12 * fabs(col->lb);
         if (lb < col->lb - eps)
         {  /* yes, it can */
            ret = 1;
            goto done;
         }
      }
      /* check if x[q] can reach its own upper bound */
      if (col->ub != +DBL_MAX)
      {  eps = 1e-9 + 1e-12 * fabs(col->ub);
         if (ub > col->ub + eps)
         {  /* yes, it can */
            ret = 1;
            goto done;
         }
      }
      /* create transformation entry */
      info = npp_push_tse(npp,
         rcv_col_sngtn2, sizeof(struct col_sngtn2));
      info->p = row->i;
      info->q = col->j;
      info->stat = 0;
      /* x[q] is implied free variable */
      col->lb = -DBL_MAX, col->ub = +DBL_MAX;
      /* since x[q] is always basic, the row p must be active; compute
         its dual value pi[p]; see (7) */
      temp = col->coef / apq->val;
      /* analyze the dual value pi[p] */
      if (temp > 0.0)
      {  /* pi[p] > 0; the row p must be active on its lower bound */
         if (row->lb != -DBL_MAX)
         {  info->stat = GLP_NL;
            row->ub = row->lb;
         }
         else
         {  if (temp > +1e-3)
            {  /* dual infeasibility */
               ret = 2;
               goto done;
            }
            xassert(row->ub != +DBL_MAX);
            info->stat = GLP_NU;
            row->lb = row->ub;
         }
      }
      else if (temp < 0.0)
      {  /* pi[p] < 0; the row p must be active on its upper bound */
         if (row->ub != +DBL_MAX)
         {  info->stat = GLP_NU;
            row->lb = row->ub;
         }
         else
         {  if (temp < -1e-3)
            {  /* dual infeasibility */
               ret = 2;
               goto done;
            }
            xassert(row->lb != -DBL_MAX);
            info->stat = GLP_NL;
            row->ub = row->lb;
         }
      }
      else
      {  /* pi[p] = 0; the row must be active on any bound */
         if (row->lb != -DBL_MAX)
         {  info->stat = GLP_NL;
            row->ub = row->lb;
         }
         else
         {  xassert(row->ub != +DBL_MAX);
            info->stat = GLP_NU;
            row->lb = row->ub;
         }
      }
      /* now x[q] can be considered as an implied slack variable */
#if 0
      npp_col_sngtn1(npp, col);
#endif
      ret = 0;
done: return ret;
}

static void rcv_col_sngtn2(NPP *npp, void *_info)
{     /* recover column singleton (implied free variable) */
      struct col_sngtn2 *info = _info;
      if (npp->sol == GLP_SOL)
      {  /* recover basic solution */
         /* the (modified) row is already recovered and must be active
            equality constraint */
         xassert(npp->r_stat[info->p] == GLP_NS);
         /* the (modified) column is already recovered and must be basic
            variable */
         xassert(npp->c_stat[info->q] == GLP_BS);
         /* set proper status of the row */
         npp->r_stat[info->p] = info->stat;
      }
      return;
}

/***********************************************************************
*  FORCING ROW
*
*  Let row p be of general kind:
*
*     l[p] <= x[p] = sum a[p,j] * x[j] <= u[p],                      (1)
*                     j
*
*     l[j] <= x[j] <= u[j].                                          (2)
*
*  If l[p] = u' or u[p] = l', where l' and u' are implied lower and
*  upper bounds of the row, the constraint forces the corresponding
*  variables x[j] to be set on their bounds in order to provide primal
*  feasibility:
*
*  if l[p] = u' and a[p,j] > 0 or if u[p] = l' and a[p,j] < 0, x[j] can
*  only be set on its upper bound u[j], and
*
*  if l[p] = u' and a[p,j] < 0 or if u[p] = l' and a[p,j] > 0, x[j] can
*  only be set on its lower bound l[j].
*
*  Being set on their bounds the variables x[j] become fixed and can be
*  removed from the problem. At the same time the row becomes redundant
*  and therefore can also be removed from the problem.
*
*  On entry to the recovering routine all x[j] are non-basic and fixed
*  while the row p is not recovered yet.
*
*  Since the corresponding basic solution is assumed to be optimal, the
*  following dual equality constraints are satisfied:
*
*     sum a[i,j] * pi[i] + lambda'[j] = c[j],                        (3)
*    i!=p
*
*  where lambda'[j] is a dual value of the column j on entry to the
*  recovering routine.
*
*  On recovering the row p an additional term that corresponds to this
*  row appears in the dual constraints (3):
*
*     sum a[i,j] * pi[i] + a[p,j] * pi[p] + lambda[j] = c[j],        (4)
*    i!=p
*
*  where pi[p] is recovered dual value of the row p, and lambda[j] is
*  recovered dual value of the column j.
*
*  From (3) and (4) it follows that:
*
*     lambda[j] = lambda'[j] - a[p,j] * pi[p].                       (5)
*
*  Now note that actually x[j] are non-fixed. Therefore, the following
*  dual feasibility condition must be satisfied for all x[j]:
*
*     lambda[j] >= 0, if x[j] was forced on its lower bound l[j],    (6)
*
*     lambda[j] <= 0, if x[j] was forced on its upper bound u[j].    (7)
*
*  Let the row p is non-active (i.e. x[p] is basic). Then pi[p] = 0 and
*  therefore lambda[j] = lambda'[j]. If the conditions (6)-(7) are
*  satisfied for all x[j], recovering is finished. Otherwise, we should
*  change (increase or decrease) pi[p] while at least one lambda[j] in
*  (5) has wrong sign. (Note that it is *always* possible to attain dual
*  feasibility by changing pi[p].) Once signs of all lambda[j] become
*  correct, there is always some lambda[q], which reaches zero last. In
*  this case the row p is active (i.e. x[p] is non-basic) with non-zero
*  dual value pi[p], all columns (except the column q) are non-basic
*  with dual values lambda[j], which are computed using the formula (5),
*  and the column q is basic with zero dual value lambda[q]. Should also
*  note that due to primal degeneracy changing pi[p] doesn't affect any
*  primal values x[p] and x[j].
*
*  RETURNS
*
*  0 - operation successful;
*  1 - cannot fix columns due to small constraint coefficients. */

struct forcing_row
{     /* forcing row */
      int p;
      /* reference number of the row */
      char stat;
      /* status assigned to the row if it becomes active constraint:
         GLP_NS - equality constraint
         GLP_NL - inequality constraint on lower bound
         GLP_NU - inequality constraint on upper bound */
      double bnd;
      /* primal value of the row (one of its bounds) */
      NPPLFX *ptr;
      /* list of non-zero coefficients a[p,j] with additional flags of
         the corresponding variables x[j]:
         GLP_NL - x[j] is fixed on its lower bound
         GLP_NU - x[j] is fixed on its upper bound */
};

static void rcv_forcing_row(NPP *npp, void *info);

int npp_forcing_row(NPP *npp, NPPROW *row, int at)
{     /* process forcing row */
      struct forcing_row *info;
      NPPCOL *col;
      NPPAIJ *aij;
      NPPLFX *lfx;
      int ret;
      double big;
      /* the row must not be empty */
      xassert(row->ptr != NULL);
      /* determine maximal magnitude of the row coefficients */
      big = 1.0;
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
         if (big < fabs(aij->val)) big = fabs(aij->val);
      /* if there are too small coefficients, fixing corresponding
         columns may be unreliable */
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  if (fabs(aij->val) < 1e-7 * big)
         {  ret = 1;
            goto done;
         }
      }
      info = npp_push_tse(npp,
         rcv_forcing_row, sizeof(struct forcing_row));
      info->p = row->i;
      if (row->lb == row->ub)
      {  /* equality constraint */
         info->stat = GLP_NS;
         info->bnd = row->lb;
      }
      else if (at == 0)
      {  /* inequality constraint; the case is l[p] = u' */
         info->stat = GLP_NL;
         xassert(row->lb != -DBL_MAX);
         info->bnd = row->lb;
      }
      else
      {  /* inequality constraint; the case is u[p] = l' */
         info->stat = GLP_NU;
         xassert(row->ub != +DBL_MAX);
         info->bnd = row->ub;
      }
      info->ptr = NULL;
      /* walk through the list of constraint coefficients of the row,
         save them, and fix the corresponding columns at appropriate
         bounds */
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  col = aij->col;
         /* save the constraint coefficient a[p,j] */
         lfx = dmp_get_atom(npp->stack, sizeof(NPPLFX));
         lfx->ref = col->j;
         lfx->flag = 0; /* will be set below */
         lfx->val = aij->val;
         lfx->next = info->ptr;
         info->ptr = lfx;
         /* the column must not be fixed */
         xassert(col->lb != col->ub);
         /* fix the column */
         if (at == 0 && aij->val < 0.0 || at != 0 && aij->val > 0.0)
         {  /* at its lower bound */
            lfx->flag = GLP_NL;
            xassert(col->lb != -DBL_MAX);
            col->ub = col->lb;
         }
         else
         {  /* at its upper bound */
            lfx->flag = GLP_NU;
            xassert(col->ub != +DBL_MAX);
            col->lb = col->ub;
         }
      }
      /* now the row is redundant */
      row->lb = -DBL_MAX, row->ub = +DBL_MAX;
      ret = 0;
done: return ret;
}

static void rcv_forcing_row(NPP *npp, void *_info)
{     /* recover forcing row */
      struct forcing_row *info = _info;
      NPPLFX *lfx, *that;
      double big, lambda, pi, temp;
      if (npp->sol == GLP_MIP)
      {  npp->r_prim[info->p] = info->bnd;
         goto done;
      }
      /* the row is not recovered yet while all corresponding columns
         were already recovered and being fixed are non-basic */
      if (npp->sol == GLP_SOL)
      {  for (lfx = info->ptr; lfx != NULL; lfx = lfx->next)
            xassert(npp->c_stat[lfx->ref] == GLP_NS);
      }
      /* choose column q, whose dual value lambda[q] has wrong sign and
         reaches zero last on changing pi[p] */
      that = NULL, big = 0.0;
      for (lfx = info->ptr; lfx != NULL; lfx = lfx->next)
      {  lambda = npp->c_dual[lfx->ref];
         temp = fabs(lambda / lfx->val);
         if (lfx->flag == GLP_NL)
         {  /* x[j] >= l[j], therefore lambda[j] >= 0 */
            if (lambda < 0.0 && big < temp) that = lfx, big = temp;
         }
         else if (lfx->flag == GLP_NU)
         {  /* x[j] <= u[j], therefore lambda[j] <= 0 */
            if (lambda > 0.0 && big < temp) that = lfx, big = temp;
         }
         else
            xassert(lfx != lfx);
      }
      /* recover the row and all corresponding columns */
      if (that == NULL)
      {  /* the row p is inactive; all columns are non-basic */
         if (npp->sol == GLP_SOL)
            npp->r_stat[info->p] = GLP_BS;
         npp->r_prim[info->p] = info->bnd;
         npp->r_dual[info->p] = 0.0;
         if (npp->sol == GLP_SOL)
         {  for (lfx = info->ptr; lfx != NULL; lfx = lfx->next)
               npp->c_stat[lfx->ref] = lfx->flag;
         }
      }
      else
      {  /* the row is active, column q is basic, and all other columns
            are non-basic */
         /* compute dual value pi[p] = lambda'[q] / a[p,q]; see (5) */
         pi = npp->c_dual[that->ref] / that->val;
         /* recover the row */
         if (npp->sol == GLP_SOL)
            npp->r_stat[info->p] = info->stat;
         npp->r_prim[info->p] = info->bnd;
         npp->r_dual[info->p] = pi;
         /* recover corresponding columns */
         for (lfx = info->ptr; lfx != NULL; lfx = lfx->next)
         {  if (lfx == that)
            {  /* recover column q, which is basic */
               if (npp->sol == GLP_SOL)
                  npp->c_stat[lfx->ref] = GLP_BS;
               npp->c_dual[lfx->ref] = 0.0;
            }
            else
            {  /* recover column j, which is non-basic */
               if (npp->sol == GLP_SOL)
                  npp->c_stat[lfx->ref] = lfx->flag;
               npp->c_dual[lfx->ref] -= lfx->val * pi;
            }
         }
      }
done: return;
}

/* eof */
