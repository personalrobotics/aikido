/* glpnpp03.c */

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

static void activate_row(NPP *npp, NPPROW *row)
{     /* make a row active */
      if (!row->temp)
      {  row->temp = 1;
         /* move the row to the beginning of the row list */
         npp_remove_row(npp, row);
         npp_insert_row(npp, row, 0);
      }
      return;
}

static void deactivate_row(NPP *npp, NPPROW *row)
{     /* make a row inactive */
      if (row->temp)
      {  row->temp = 0;
         /* move the row to the end of the row list */
         npp_remove_row(npp, row);
         npp_insert_row(npp, row, 1);
      }
      return;
}

static void activate_col(NPP *npp, NPPCOL *col)
{     /* make a column active */
      if (!col->temp)
      {  col->temp = 1;
         /* move the column to the beginning of the column list */
         npp_remove_col(npp, col);
         npp_insert_col(npp, col, 0);
      }
      return;
}

static void deactivate_col(NPP *npp, NPPCOL *col)
{     /* make a column inactive */
      if (col->temp)
      {  col->temp = 0;
         /* move the column to the end of the column list */
         npp_remove_col(npp, col);
         npp_insert_col(npp, col, 1);
      }
      return;
}

/***********************************************************************
*  GENERAL ROW ANALYSIS
*
*  Let row p be of general kind:
*
*     l[p] <= x[p] = sum a[p,j] * x[j] <= u[p],                      (1)
*                     j
*
*     l[j] <= x[j] <= u[j].                                          (2)
*
*  The analysis is based on implied lower and upper bounds l' and u' of
*  the primal value x[p]:
*
*                       ( l[j], if a[p,j] > 0 )
*     l' = sum a[p,j] * <                     > ,                    (3)
*           j           ( u[j], if a[p,j] < 0 )
*
*                       ( u[j], if a[p,j] > 0 )
*     u' = sum a[p,j] * <                     > .                    (4)
*           j           ( l[j], if a[p,j] < 0 )
*
*  If l' > u[p] or u' < l[p], the row is primal infeasible.
*
*  If l' = u[p], all the variables x[j] with non-zero coefficients in
*  the row p can be fixed on their bounds as follows:
*
*     if a[p,j] > 0, x[j] is fixed on l[j];                          (5)
*
*     if a[p,j] < 0, x[j] is fixed on u[j].                          (6)
*
*  If u' = l[p], all the variables x[j] with non-zero coefficients in
*  the row p can be fixed on their bounds as follows:
*
*     if a[p,j] > 0, x[j] is fixed on u[j];                          (7)
*
*     if a[p,j] < 0, x[j] is fixed on l[j].                          (8)
*
*  In both cases l' = u[p] and u' = l[p] after fixing variables the row
*  becomes redundant and therefore can be removed from the problem.
*
*  RETURNS
*
*  0x?0 - lower bound cannot be active (l[p] < l')
*  0x?1 - lower bound can be active (l[p] >= l')
*  0x?2 - forcing lower bound (l' = u[p])
*  0x0? - upper bound cannot be active (u[p] > u')
*  0x1? - upper bound can be active (u[p] <= u')
*  0x2? - forcing upper bound (u' = l[p])
*  0x33 - primal infeasibility detected (l' > u[p] or u' < l[p]). */

static int analyze_row(NPP *npp, NPPROW *row)
{     /* general row analysis */
      NPPAIJ *aij;
      int ret = 0x00;
      double fmin, fmax, eps;
      xassert(npp == npp);
      /* compute implied lower bound l'; see (3) */
      fmin = 0.0;
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  if (aij->val > 0.0)
         {  if (aij->col->lb == -DBL_MAX)
            {  fmin = -DBL_MAX;
               break;
            }
            fmin += aij->val * aij->col->lb;
         }
         else /* aij->val < 0.0 */
         {  if (aij->col->ub == +DBL_MAX)
            {  fmin = -DBL_MAX;
               break;
            }
            fmin += aij->val * aij->col->ub;
         }
      }
      /* compute implied upper bound u'; see (4) */
      fmax = 0.0;
      for (aij = row->ptr; aij != NULL; aij = aij->r_next)
      {  if (aij->val > 0.0)
         {  if (aij->col->ub == +DBL_MAX)
            {  fmax = +DBL_MAX;
               break;
            }
            fmax += aij->val * aij->col->ub;
         }
         else /* aij->val < 0.0 */
         {  if (aij->col->lb == -DBL_MAX)
            {  fmax = +DBL_MAX;
               break;
            }
            fmax += aij->val * aij->col->lb;
         }
      }
      /* bounds of variables are assumed correct, so fmin <= fmax */
      /* check primal feasibility */
      if (row->ub != +DBL_MAX)
      {  eps = 1e-3 + 1e-6 * fabs(row->ub);
         if (fmin > row->ub + eps)
         {  ret = 0x33;
            goto done;
         }
      }
      if (row->lb != -DBL_MAX)
      {  eps = 1e-3 + 1e-6 * fabs(row->lb);
         if (fmax < row->lb - eps)
         {  ret = 0x33;
            goto done;
         }
      }
      /* check if the lower bound can be active */
      if (row->lb != -DBL_MAX)
      {  eps = 1e-9 + 1e-12 * fabs(row->lb);
         if (fmin < row->lb - eps)
         {  /* check if the lower bound is forcing */
            if (fmax > row->lb + eps)
               ret |= 0x01;
            else
               ret |= 0x02;
         }
      }
      /* check if the upper bound can be active */
      if (row->ub != +DBL_MAX)
      {  eps = 1e-9 + 1e-12 * fabs(row->ub);
         if (fmax > row->ub + eps)
         {  /* check if the upper bound is forcing */
            if (fmin < row->ub - eps)
               ret |= 0x10;
            else
               ret |= 0x20;
         }
      }
done: return ret;
}

struct csa
{     NPP *npp;
      int cnt_free_row;
      int cnt_empty_row;
      int cnt_row_sngtn1;
      int cnt_row_sngtn2;
      int cnt_forcing_row;
      int cnt_fixed_col;
      int cnt_empty_col;
      int cnt_col_sngtn1;
      int cnt_col_sngtn2;
};

static int preprocess_row(struct csa *csa, NPPROW *row)
{     /* preprocess a row (constraint) */
      NPP *npp = csa->npp;
      NPPCOL *col;
      NPPAIJ *aij, *next_aij, *aaa;
      int ret;
      /* the row must not be free */
      xassert(!(row->lb == -DBL_MAX && row->ub == +DBL_MAX));
      if (row->ptr == NULL)
      {  /* empty row */
         if (npp_empty_row(npp, row))
         {  ret = GLP_ENOPFS;
            goto done;
         }
         csa->cnt_empty_row++;
         /* the row has been deleted */
         ret = 0;
         goto done;
      }
      if (row->ptr->r_next == NULL)
      {  /* row singleton */
         col = row->ptr->col;
         if (row->lb == row->ub)
         {  /* equality constraint */
            if (npp_row_sngtn1(npp, row))
            {  ret = GLP_ENOPFS;
               goto done;
            }
            csa->cnt_row_sngtn1++;
            /* the row has been deleted, the column has been fixed */
            /* activate rows affected by the column */
            for (aij = col->ptr; aij != NULL; aij = aij->c_next)
               activate_row(npp, aij->row);
            /* process the fixed column */
            npp_fixed_col(npp, col);
            csa->cnt_fixed_col++;
            /* the column has been deleted */
         }
         else
         {  /* inequality constraint */
            ret = npp_row_sngtn2(npp, row);
            xassert(0 <= ret && ret <= 4);
            if (ret == 4)
            {  ret = GLP_ENOPFS;
               goto done;
            }
            csa->cnt_row_sngtn2++;
            /* the row has been deleted */
            /* activate the column, since its length decreased */
            activate_col(npp, col);
            if (ret == 2 || ret == 3)
            {  /* bounds of the column changed significantly, or the
                  column has been fixed */
               /* activate rows affected by the column */
               for (aij = col->ptr; aij != NULL; aij = aij->c_next)
                  activate_row(npp, aij->row);
               if (ret == 3)
               {  /* process the fixed column */
                  npp_fixed_col(npp, col);
                  csa->cnt_fixed_col++;
                  /* the column has been deleted */
               }
            }
         }
         ret = 0;
         goto done;
      }
      /* general row analysis */
      ret = analyze_row(npp, row);
      if (ret == 0x33)
      {  ret = GLP_ENOPFS;
         goto done;
      }
      if (ret == 0x00)
      {  /* bounds of the row cannot be active; make the row free */
         row->lb = -DBL_MAX, row->ub = +DBL_MAX;
         /* activate columns affected by the row */
         for (aij = row->ptr; aij != NULL; aij = aij->r_next)
            activate_col(npp, aij->col);
         /* process the free row */
         npp_free_row(npp, row);
         csa->cnt_free_row++;
         /* the row has been deleted */
         ret = 0;
         goto done;
      }
      if ((ret & 0x0F) == 0x02 || (ret & 0xF0) == 0x20)
      {  /* forcing row */
         if (npp_forcing_row(npp, row, (ret & 0x0F) == 0x02 ? 0 : 1))
            goto skip;
         csa->cnt_forcing_row++;
         /* now the row is free and corresponding columns are fixed */
         for (aij = row->ptr; aij != NULL; aij = next_aij)
         {  col = aij->col;
            next_aij = aij->r_next;
            /* activate rows affected by current column */
            for (aaa = col->ptr; aaa != NULL; aaa = aaa->c_next)
               activate_row(npp, aaa->row);
            /* process current fixed column */
            npp_fixed_col(npp, col);
            csa->cnt_fixed_col++;
            /* current column has been deleted */
         }
         /* process the free row */
         npp_free_row(npp, row);
         csa->cnt_free_row++;
         /* the row has been deleted */
         ret = 0;
         goto done;
      }
skip: ;
      ret = 0;
done: return ret;
}

static int preprocess_col(struct csa *csa, NPPCOL *col)
{     /* preprocess a column (variable) */
      NPP *npp = csa->npp;
      NPPROW *row;
      NPPAIJ *aij;
      int ret;
      /* the column must not be fixed */
      xassert(col->lb != col->ub);
      if (col->ptr == NULL)
      {  /* empty column */
         if (npp_empty_col(npp, col))
         {  ret = GLP_ENODFS;
            goto done;
         }
         csa->cnt_empty_col++;
         /* the column has been deleted */
         ret = 0;
         goto done;
      }
      if (col->ptr->c_next == NULL)
      {  /* column singleton */
         if (npp->sol == GLP_MIP && col->kind == GLP_IV) goto skip;
         row = col->ptr->row;
         if (row->lb == row->ub)
isv:     {  /* implied slack variable */
            npp_col_sngtn1(npp, col);
            csa->cnt_col_sngtn1++;
            /* the column has been deleted */
            /* activate the row, since its length decreased */
            activate_row(npp, row);
            if (row->lb == -DBL_MAX && row->ub == +DBL_MAX)
            {  /* the row was made free */
               /* activate columns affected by the row */
               for (aij = row->ptr; aij != NULL; aij = aij->r_next)
                  activate_col(npp, aij->col);
               /* process the free row */
               npp_free_row(npp, row);
               csa->cnt_free_row++;
               /* the row has been deleted */
            }
         }
         else
         {  /* implied free variable */
            ret = npp_col_sngtn2(npp, col);
            xassert(0 <= ret && ret <= 2);
            if (ret == 2)
            {  ret = GLP_ENODFS;
               goto done;
            }
            if (ret == 0)
            {  /* the row was made equality constraint, and the column
                  was made free */
               xassert(row->lb == row->ub);
               xassert(col->lb == -DBL_MAX && col->ub == +DBL_MAX);
               csa->cnt_col_sngtn2++;
               csa->cnt_col_sngtn1--;
               goto isv;
            }
         }
         ret = 0;
         goto done;
      }
skip: ;
      ret = 0;
done: return ret;
}

int npp_preprocess(NPP *npp)
{     /* preprocess LP/MIP instance */
      struct csa _csa, *csa = &_csa;
      NPPROW *row, *next_row;
      NPPCOL *col, *next_col;
      int running, ret;
      /* the preprocessor uses two sets: a set of active rows and a set
         of active columns; active rows/columns have non-zero flag (the
         field temp in NPPROW/NPPCOL) and are always placed in the
         beginning of the row/column list; inactive rows/columns have
         zero flag and follow active rows/columns in the row/column
         list; initially the active sets contain all original rows and
         columns, except free rows and fixed columns, which are removed
         as they appear and, thus, can never be in the active sets;
         once a next row or column has been selected for processing, it
         is removed from the corresponding active set; if an operation
         applied may affect other rows or columns, they are activated
         for further processing */
      csa->npp = npp;
      csa->cnt_free_row = 0;
      csa->cnt_empty_row = 0;
      csa->cnt_row_sngtn1 = 0;
      csa->cnt_row_sngtn2 = 0;
      csa->cnt_forcing_row = 0;
      csa->cnt_fixed_col = 0;
      csa->cnt_empty_col = 0;
      csa->cnt_col_sngtn1 = 0;
      csa->cnt_col_sngtn2 = 0;
      /* process free rows */
      for (row = npp->r_head; row != NULL; row = next_row)
      {  next_row = row->next;
         if (row->lb == -DBL_MAX && row->ub == +DBL_MAX)
         {  npp_free_row(npp, row);
            csa->cnt_free_row++;
            /* the row has been deleted */
         }
      }
      /* process fixed columns */
      for (col = npp->c_head; col != NULL; col = next_col)
      {  next_col = col->next;
         if (col->lb == col->ub)
         {  npp_fixed_col(npp, col);
            csa->cnt_fixed_col++;
            /* the column has been deleted */
         }
      }
      /* activate all remaining rows and columns */
      for (row = npp->r_head; row != NULL; row = row->next)
         row->temp = 1;
      for (col = npp->c_head; col != NULL; col = col->next)
         col->temp = 1;
      /* main preprocessing loop starts here */
      running = 1;
      while (running)
      {  running = 0;
         /* preprocess all active rows */
         for (;;)
         {  row = npp->r_head;
            if (row == NULL || !row->temp) break;
            deactivate_row(npp, row);
            ret = preprocess_row(csa, row);
            if (ret != 0) goto done;
            running = 1;
         }
         /* preprocess all active columns */
         for (;;)
         {  col = npp->c_head;
            if (col == NULL || !col->temp) break;
            deactivate_col(npp, col);
            ret = preprocess_col(csa, col);
            if (ret != 0) goto done;
            running = 1;
         }
      }
      /* all seems ok */
      ret = 0;
#if 0
      xprintf("cnt_free_row    = %d\n", csa->cnt_free_row);
      xprintf("cnt_empty_row   = %d\n", csa->cnt_empty_row);
      xprintf("cnt_row_sngtn1  = %d\n", csa->cnt_row_sngtn1);
      xprintf("cnt_row_sngtn2  = %d\n", csa->cnt_row_sngtn2);
      xprintf("cnt_forcing_row = %d\n", csa->cnt_forcing_row);
      xprintf("cnt_fixed_col   = %d\n", csa->cnt_fixed_col);
      xprintf("cnt_empty_col   = %d\n", csa->cnt_empty_col);
      xprintf("cnt_col_sngtn1  = %d\n", csa->cnt_col_sngtn1);
      xprintf("cnt_col_sngtn2  = %d\n", csa->cnt_col_sngtn2);
#endif
done: xassert(ret == 0 || ret == GLP_ENOPFS || ret == GLP_ENODFS);
      return ret;
}

/* eof */
