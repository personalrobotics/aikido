/* glpnpp01.c */

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

NPP *npp_create_wksp(void)
{     /* create LP/MIP preprocessor workspace */
      NPP *npp;
      npp = xmalloc(sizeof(NPP));
      npp->orig_dir = 0;
      npp->orig_m = npp->orig_n = npp->orig_nnz = 0;
      npp->pool = dmp_create_pool();
      npp->name = npp->obj = NULL;
      npp->c0 = 0.0;
      npp->nrows = npp->ncols = 0;
      npp->r_head = npp->r_tail = NULL;
      npp->c_head = npp->c_tail = NULL;
      npp->stack = dmp_create_pool();
      npp->top = NULL;
      npp->m = npp->n = npp->nnz = 0;
      npp->row_ref = npp->col_ref = NULL;
      npp->sol = npp->scaling = 0;
      npp->p_stat = npp->d_stat = npp->t_stat = npp->i_stat = 0;
      npp->r_stat = NULL;
      npp->r_prim = npp->r_dual = NULL;
      npp->c_stat = NULL;
      npp->c_prim = npp->c_dual = NULL;
      return npp;
}

void npp_insert_row(NPP *npp, NPPROW *row, int where)
{     /* insert row to the row list */
      if (where == 0)
      {  /* insert row to the beginning of the row list */
         row->prev = NULL;
         row->next = npp->r_head;
         if (row->next == NULL)
            npp->r_tail = row;
         else
            row->next->prev = row;
         npp->r_head = row;
      }
      else
      {  /* insert row to the end of the row list */
         row->prev = npp->r_tail;
         row->next = NULL;
         if (row->prev == NULL)
            npp->r_head = row;
         else
            row->prev->next = row;
         npp->r_tail = row;
      }
      return;
}

void npp_remove_row(NPP *npp, NPPROW *row)
{     /* remove row from the row list */
      if (row->prev == NULL)
         npp->r_head = row->next;
      else
         row->prev->next = row->next;
      if (row->next == NULL)
         npp->r_tail = row->prev;
      else
         row->next->prev = row->prev;
      return;
}

void npp_insert_col(NPP *npp, NPPCOL *col, int where)
{     /* insert column to the column list */
      if (where == 0)
      {  /* insert column to the beginning of the column list */
         col->prev = NULL;
         col->next = npp->c_head;
         if (col->next == NULL)
            npp->c_tail = col;
         else
            col->next->prev = col;
         npp->c_head = col;
      }
      else
      {  /* insert column to the end of the column list */
         col->prev = npp->c_tail;
         col->next = NULL;
         if (col->prev == NULL)
            npp->c_head = col;
         else
            col->prev->next = col;
         npp->c_tail = col;
      }
      return;
}

void npp_remove_col(NPP *npp, NPPCOL *col)
{     /* remove column from the column list */
      if (col->prev == NULL)
         npp->c_head = col->next;
      else
         col->prev->next = col->next;
      if (col->next == NULL)
         npp->c_tail = col->prev;
      else
         col->next->prev = col->prev;
      return;
}

NPPROW *npp_add_row(NPP *npp)
{     /* add new row to the transformed problem */
      NPPROW *row;
      row = dmp_get_atom(npp->pool, sizeof(NPPROW));
      row->i = ++(npp->nrows);
      row->name = NULL;
      row->lb = -DBL_MAX, row->ub = +DBL_MAX;
      row->ptr = NULL;
      row->temp = 0;
      npp_insert_row(npp, row, 1);
      return row;
}

NPPCOL *npp_add_col(NPP *npp)
{     /* add new column to the transformed problem */
      NPPCOL *col;
      col = dmp_get_atom(npp->pool, sizeof(NPPCOL));
      col->j = ++(npp->ncols);
      col->name = NULL;
      col->kind = GLP_CV;
      col->lb = col->ub = col->coef = 0.0;
      col->ptr = NULL;
      col->temp = 0;
      npp_insert_col(npp, col, 1);
      return col;
}

NPPAIJ *npp_add_aij(NPP *npp, NPPROW *row, NPPCOL *col, double val)
{     /* add new element to the constraint matrix */
      NPPAIJ *aij;
      aij = dmp_get_atom(npp->pool, sizeof(NPPAIJ));
      aij->row = row;
      aij->col = col;
      aij->val = val;
      aij->r_prev = NULL;
      aij->r_next = row->ptr;
      aij->c_prev = NULL;
      aij->c_next = col->ptr;
      if (aij->r_next != NULL)
         aij->r_next->r_prev = aij;
      if (aij->c_next != NULL)
         aij->c_next->c_prev = aij;
      row->ptr = col->ptr = aij;
      return aij;
}

void *npp_push_tse(NPP *npp, void (*func)(NPP *npp, void *info),
      int size)
{     /* push new entry to the transformation stack */
      NPPTSE *tse;
      tse = dmp_get_atom(npp->stack, sizeof(NPPTSE));
      tse->func = func;
      tse->info = dmp_get_atom(npp->stack, size);
      tse->link = npp->top;
      npp->top = tse;
      return tse->info;
}

void npp_del_row(NPP *npp, NPPROW *row)
{     /* remove row from the transformed problem */
      NPPAIJ *aij;
      if (row->name != NULL)
         dmp_free_atom(npp->pool, row->name, strlen(row->name)+1);
      while (row->ptr != NULL)
      {  aij = row->ptr;
         row->ptr = aij->r_next;
         if (aij->c_prev == NULL)
            aij->col->ptr = aij->c_next;
         else
            aij->c_prev->c_next = aij->c_next;
         if (aij->c_next == NULL)
            ;
         else
            aij->c_next->c_prev = aij->c_prev;
         dmp_free_atom(npp->pool, aij, sizeof(NPPAIJ));
      }
      npp_remove_row(npp, row);
      dmp_free_atom(npp->pool, row, sizeof(NPPROW));
      return;
}

void npp_del_col(NPP *npp, NPPCOL *col)
{     /* remove column from the transformed problem */
      NPPAIJ *aij;
      if (col->name != NULL)
         dmp_free_atom(npp->pool, col->name, strlen(col->name)+1);
      while (col->ptr != NULL)
      {  aij = col->ptr;
         col->ptr = aij->c_next;
         if (aij->r_prev == NULL)
            aij->row->ptr = aij->r_next;
         else
            aij->r_prev->r_next = aij->r_next;
         if (aij->r_next == NULL)
            ;
         else
            aij->r_next->r_prev = aij->r_prev;
         dmp_free_atom(npp->pool, aij, sizeof(NPPAIJ));
      }
      npp_remove_col(npp, col);
      dmp_free_atom(npp->pool, col, sizeof(NPPCOL));
      return;
}

void npp_load_prob(NPP *npp, glp_prob *orig, int names, int sol,
      int scaling)
{     /* load original problem into the preprocessor workspace */
      int m = orig->m;
      int n = orig->n;
      NPPROW **link;
      int i, j;
      double dir;
      xassert(names == GLP_OFF || names == GLP_ON);
      xassert(sol == GLP_SOL || sol == GLP_IPT || sol == GLP_MIP);
      xassert(scaling == GLP_OFF || scaling == GLP_ON);
      if (sol == GLP_MIP) xassert(!scaling);
      npp->orig_dir = orig->dir;
      if (npp->orig_dir == GLP_MIN)
         dir = +1.0;
      else if (npp->orig_dir == GLP_MAX)
         dir = -1.0;
      else
         xassert(npp != npp);
      npp->orig_m = m;
      npp->orig_n = n;
      npp->orig_nnz = orig->nnz;
      if (names && orig->name != NULL)
      {  npp->name = dmp_get_atom(npp->pool, strlen(orig->name)+1);
         strcpy(npp->name, orig->name);
      }
      if (names && orig->obj != NULL)
      {  npp->obj = dmp_get_atom(npp->pool, strlen(orig->obj)+1);
         strcpy(npp->obj, orig->obj);
      }
      npp->c0 = dir * orig->c0;
      /* load rows */
      link = xcalloc(1+m, sizeof(NPPROW *));
      for (i = 1; i <= m; i++)
      {  GLPROW *rrr = orig->row[i];
         NPPROW *row;
         link[i] = row = npp_add_row(npp);
         xassert(row->i == i);
         if (names && rrr->name != NULL)
         {  row->name = dmp_get_atom(npp->pool, strlen(rrr->name)+1);
            strcpy(row->name, rrr->name);
         }
         if (!scaling)
         {  if (rrr->type == GLP_FR)
               row->lb = -DBL_MAX, row->ub = +DBL_MAX;
            else if (rrr->type == GLP_LO)
               row->lb = rrr->lb, row->ub = +DBL_MAX;
            else if (rrr->type == GLP_UP)
               row->lb = -DBL_MAX, row->ub = rrr->ub;
            else if (rrr->type == GLP_DB)
               row->lb = rrr->lb, row->ub = rrr->ub;
            else if (rrr->type == GLP_FX)
               row->lb = row->ub = rrr->lb;
            else
               xassert(rrr != rrr);
         }
         else
         {  double rii = rrr->rii;
            if (rrr->type == GLP_FR)
               row->lb = -DBL_MAX, row->ub = +DBL_MAX;
            else if (rrr->type == GLP_LO)
               row->lb = rrr->lb * rii, row->ub = +DBL_MAX;
            else if (rrr->type == GLP_UP)
               row->lb = -DBL_MAX, row->ub = rrr->ub * rii;
            else if (rrr->type == GLP_DB)
               row->lb = rrr->lb * rii, row->ub = rrr->ub * rii;
            else if (rrr->type == GLP_FX)
               row->lb = row->ub = rrr->lb * rii;
            else
               xassert(rrr != rrr);
         }
      }
      /* load columns and constraint coefficients */
      for (j = 1; j <= n; j++)
      {  GLPCOL *ccc = orig->col[j];
         GLPAIJ *aaa;
         NPPCOL *col;
         col = npp_add_col(npp);
         xassert(col->j == j);
         if (names && ccc->name != NULL)
         {  col->name = dmp_get_atom(npp->pool, strlen(ccc->name)+1);
            strcpy(col->name, ccc->name);
         }
         if (sol == GLP_MIP)
            col->kind = ccc->kind;
         if (!scaling)
         {  if (ccc->type == GLP_FR)
               col->lb = -DBL_MAX, col->ub = +DBL_MAX;
            else if (ccc->type == GLP_LO)
               col->lb = ccc->lb, col->ub = +DBL_MAX;
            else if (ccc->type == GLP_UP)
               col->lb = -DBL_MAX, col->ub = ccc->ub;
            else if (ccc->type == GLP_DB)
               col->lb = ccc->lb, col->ub = ccc->ub;
            else if (ccc->type == GLP_FX)
               col->lb = col->ub = ccc->lb;
            else
               xassert(ccc != ccc);
            col->coef = dir * ccc->coef;
            for (aaa = ccc->ptr; aaa != NULL; aaa = aaa->c_next)
               npp_add_aij(npp, link[aaa->row->i], col, aaa->val);
         }
         else
         {  double sjj = ccc->sjj;
            if (ccc->type == GLP_FR)
               col->lb = -DBL_MAX, col->ub = +DBL_MAX;
            else if (ccc->type == GLP_LO)
               col->lb = ccc->lb / sjj, col->ub = +DBL_MAX;
            else if (ccc->type == GLP_UP)
               col->lb = -DBL_MAX, col->ub = ccc->ub / sjj;
            else if (ccc->type == GLP_DB)
               col->lb = ccc->lb / sjj, col->ub = ccc->ub / sjj;
            else if (ccc->type == GLP_FX)
               col->lb = col->ub = ccc->lb / sjj;
            else
               xassert(ccc != ccc);
            col->coef = dir * ccc->coef * sjj;
            for (aaa = ccc->ptr; aaa != NULL; aaa = aaa->c_next)
               npp_add_aij(npp, link[aaa->row->i], col,
                  aaa->row->rii * aaa->val * sjj);
         }
      }
      xfree(link);
      /* keep solution indicator and scaling option */
      npp->sol = sol;
      npp->scaling = scaling;
      return;
}

void npp_build_prob(NPP *npp, glp_prob *prob)
{     /* build resultant (preprocessed) problem */
      NPPROW *row;
      NPPCOL *col;
      NPPAIJ *aij;
      int i, j, type, len, *ind;
      double dir, *val;
      glp_erase_prob(prob);
      glp_set_prob_name(prob, npp->name);
      glp_set_obj_name(prob, npp->obj);
      glp_set_obj_dir(prob, npp->orig_dir);
      if (npp->orig_dir == GLP_MIN)
         dir = +1.0;
      else if (npp->orig_dir == GLP_MAX)
         dir = -1.0;
      else
         xassert(npp != npp);
      glp_set_obj_coef(prob, 0, dir * npp->c0);
      /* build rows */
      for (row = npp->r_head; row != NULL; row = row->next)
      {  row->temp = i = glp_add_rows(prob, 1);
         glp_set_row_name(prob, i, row->name);
         if (row->lb == -DBL_MAX && row->ub == +DBL_MAX)
            type = GLP_FR;
         else if (row->ub == +DBL_MAX)
            type = GLP_LO;
         else if (row->lb == -DBL_MAX)
            type = GLP_UP;
         else if (row->lb != row->ub)
            type = GLP_DB;
         else
            type = GLP_FX;
         glp_set_row_bnds(prob, i, type, row->lb, row->ub);
      }
      /* build columns and the constraint matrix */
      ind = xcalloc(1+prob->m, sizeof(int));
      val = xcalloc(1+prob->m, sizeof(double));
      for (col = npp->c_head; col != NULL; col = col->next)
      {  j = glp_add_cols(prob, 1);
         glp_set_col_name(prob, j, col->name);
         glp_set_col_kind(prob, j, col->kind);
         if (col->lb == -DBL_MAX && col->ub == +DBL_MAX)
            type = GLP_FR;
         else if (col->ub == +DBL_MAX)
            type = GLP_LO;
         else if (col->lb == -DBL_MAX)
            type = GLP_UP;
         else if (col->lb != col->ub)
            type = GLP_DB;
         else
            type = GLP_FX;
         glp_set_col_bnds(prob, j, type, col->lb, col->ub);
         glp_set_obj_coef(prob, j, dir * col->coef);
         len = 0;
         for (aij = col->ptr; aij != NULL; aij = aij->c_next)
         {  len++;
            ind[len] = aij->row->temp;
            val[len] = aij->val;
         }
         glp_set_mat_col(prob, j, len, ind, val);
      }
      xfree(ind);
      xfree(val);
      /* resultant problem has been built */
      npp->m = prob->m;
      npp->n = prob->n;
      npp->nnz = prob->nnz;
      npp->row_ref = xcalloc(1+npp->m, sizeof(int));
      npp->col_ref = xcalloc(1+npp->n, sizeof(int));
      for (row = npp->r_head, i = 0; row != NULL; row = row->next)
         npp->row_ref[++i] = row->i;
      for (col = npp->c_head, j = 0; col != NULL; col = col->next)
         npp->col_ref[++j] = col->j;
      /* transformed problem segment is no longer needed */
      dmp_delete_pool(npp->pool), npp->pool = NULL;
      npp->name = npp->obj = NULL;
      npp->c0 = 0.0;
      npp->r_head = npp->r_tail = NULL;
      npp->c_head = npp->c_tail = NULL;
      return;
}

void npp_postprocess(NPP *npp, glp_prob *prob)
{     /* postprocess solution from the resultant problem */
      GLPROW *row;
      GLPCOL *col;
      NPPTSE *tse;
      int i, j, k;
      double dir;
      xassert(npp->orig_dir == prob->dir);
      if (npp->orig_dir == GLP_MIN)
         dir = +1.0;
      else if (npp->orig_dir == GLP_MAX)
         dir = -1.0;
      else
         xassert(npp != npp);
      xassert(npp->m == prob->m);
      xassert(npp->n == prob->n);
      xassert(npp->nnz == prob->nnz);
      /* copy solution status */
      if (npp->sol == GLP_SOL)
      {  npp->p_stat = prob->pbs_stat;
         npp->d_stat = prob->dbs_stat;
      }
      else if (npp->sol == GLP_IPT)
         npp->t_stat = prob->ipt_stat;
      else if (npp->sol == GLP_MIP)
         npp->i_stat = prob->mip_stat;
      else
         xassert(npp != npp);
      /* allocate solution arrays */
      if (npp->sol == GLP_SOL)
      {  if (npp->r_stat == NULL)
            npp->r_stat = xcalloc(1+npp->nrows, sizeof(char));
         for (i = 1; i <= npp->nrows; i++)
            npp->r_stat[i] = 0;
         if (npp->c_stat == NULL)
            npp->c_stat = xcalloc(1+npp->ncols, sizeof(char));
         for (j = 1; j <= npp->ncols; j++)
            npp->c_stat[j] = 0;
      }
      if (npp->r_prim == NULL)
         npp->r_prim = xcalloc(1+npp->nrows, sizeof(double));
      for (i = 1; i <= npp->nrows; i++)
         npp->r_prim[i] = DBL_MAX;
      if (npp->c_prim == NULL)
         npp->c_prim = xcalloc(1+npp->ncols, sizeof(double));
      for (j = 1; j <= npp->ncols; j++)
         npp->c_prim[j] = DBL_MAX;
      if (npp->sol != GLP_MIP)
      {  if (npp->r_dual == NULL)
            npp->r_dual = xcalloc(1+npp->nrows, sizeof(double));
         for (i = 1; i <= npp->nrows; i++)
            npp->r_dual[i] = DBL_MAX;
         if (npp->c_dual == NULL)
            npp->c_dual = xcalloc(1+npp->ncols, sizeof(double));
         for (j = 1; j <= npp->ncols; j++)
            npp->c_dual[j] = DBL_MAX;
      }
      /* copy solution components from the resultant problem */
      if (npp->sol == GLP_SOL)
      {  for (i = 1; i <= npp->m; i++)
         {  row = prob->row[i];
            k = npp->row_ref[i];
            npp->r_stat[k] = (char)row->stat;
            npp->r_prim[k] = row->prim;
            npp->r_dual[k] = dir * row->dual;
         }
         for (j = 1; j <= npp->n; j++)
         {  col = prob->col[j];
            k = npp->col_ref[j];
            npp->c_stat[k] = (char)col->stat;
            npp->c_prim[k] = col->prim;
            npp->c_dual[k] = dir * col->dual;
         }
      }
      else if (npp->sol == GLP_IPT)
      {  for (i = 1; i <= npp->m; i++)
         {  row = prob->row[i];
            k = npp->row_ref[i];
            npp->r_prim[k] = row->pval;
            npp->r_dual[k] = dir * row->dval;
         }
         for (j = 1; j <= npp->n; j++)
         {  col = prob->col[j];
            k = npp->col_ref[j];
            npp->c_prim[k] = col->pval;
            npp->c_dual[k] = dir * col->dval;
         }
      }
      else if (npp->sol == GLP_MIP)
      {  for (i = 1; i <= npp->m; i++)
         {  row = prob->row[i];
            k = npp->row_ref[i];
            npp->r_prim[k] = row->mipx;
         }
         for (j = 1; j <= npp->n; j++)
         {  col = prob->col[j];
            k = npp->col_ref[j];
            npp->c_prim[k] = col->mipx;
         }
      }
      else
         xassert(npp != npp);
      /* perform postprocessing to construct solution to the original
         problem */
      for (tse = npp->top; tse != NULL; tse = tse->link)
      {  xassert(tse->func != NULL);
         tse->func(npp, tse->info);
      }
      return;
}

void npp_unload_sol(NPP *npp, glp_prob *orig)
{     /* store solution to the original problem */
      GLPROW *row;
      GLPCOL *col;
      int i, j;
      double dir;
      xassert(npp->orig_dir == orig->dir);
      if (npp->orig_dir == GLP_MIN)
         dir = +1.0;
      else if (npp->orig_dir == GLP_MAX)
         dir = -1.0;
      else
         xassert(npp != npp);
      xassert(npp->orig_m == orig->m);
      xassert(npp->orig_n == orig->n);
      xassert(npp->orig_nnz == orig->nnz);
      if (npp->sol == GLP_SOL)
      {  /* store basic solution */
         orig->valid = 0;
         orig->pbs_stat = npp->p_stat;
         orig->dbs_stat = npp->d_stat;
         orig->obj_val = orig->c0;
         orig->some = 0;
         for (i = 1; i <= orig->m; i++)
         {  row = orig->row[i];
            row->stat = npp->r_stat[i];
            if (!npp->scaling)
            {  row->prim = npp->r_prim[i];
               row->dual = dir * npp->r_dual[i];
            }
            else
            {  row->prim = npp->r_prim[i] / row->rii;
               row->dual = dir * npp->r_dual[i] * row->rii;
            }
            if (row->stat == GLP_BS)
               row->dual = 0.0;
            else if (row->stat == GLP_NL)
            {  xassert(row->type == GLP_LO || row->type == GLP_DB);
               row->prim = row->lb;
            }
            else if (row->stat == GLP_NU)
            {  xassert(row->type == GLP_UP || row->type == GLP_DB);
               row->prim = row->ub;
            }
            else if (row->stat == GLP_NF)
            {  xassert(row->type == GLP_FR);
               row->prim = 0.0;
            }
            else if (row->stat == GLP_NS)
            {  xassert(row->type == GLP_FX);
               row->prim = row->lb;
            }
            else
               xassert(row != row);
         }
         for (j = 1; j <= orig->n; j++)
         {  col = orig->col[j];
            col->stat = npp->c_stat[j];
            if (!npp->scaling)
            {  col->prim = npp->c_prim[j];
               col->dual = dir * npp->c_dual[j];
            }
            else
            {  col->prim = npp->c_prim[j] * col->sjj;
               col->dual = dir * npp->c_dual[j] / col->sjj;
            }
            if (col->stat == GLP_BS)
               col->dual = 0.0;
            else if (col->stat == GLP_NL)
            {  xassert(col->type == GLP_LO || col->type == GLP_DB);
               col->prim = col->lb;
            }
            else if (col->stat == GLP_NU)
            {  xassert(col->type == GLP_UP || col->type == GLP_DB);
               col->prim = col->ub;
            }
            else if (col->stat == GLP_NF)
            {  xassert(col->type == GLP_FR);
               col->prim = 0.0;
            }
            else if (col->stat == GLP_NS)
            {  xassert(col->type == GLP_FX);
               col->prim = col->lb;
            }
            else
               xassert(col != col);
            orig->obj_val += col->coef * col->prim;
         }
      }
      else if (npp->sol == GLP_IPT)
      {  /* store interior-point solution */
         orig->ipt_stat = npp->t_stat;
         orig->ipt_obj = orig->c0;
         for (i = 1; i <= orig->m; i++)
         {  row = orig->row[i];
            if (!npp->scaling)
            {  row->pval = npp->r_prim[i];
               row->dval = dir * npp->r_dual[i];
            }
            else
            {  row->pval = npp->r_prim[i] / row->rii;
               row->dval = dir * npp->r_dual[i] * row->rii;
            }
         }
         for (j = 1; j <= orig->n; j++)
         {  col = orig->col[j];
            if (!npp->scaling)
            {  col->pval = npp->c_prim[j];
               col->dval = dir * npp->c_dual[j];
            }
            else
            {  col->pval = npp->c_prim[j] * col->sjj;
               col->dval = dir * npp->c_dual[j] / col->sjj;
            }
            orig->ipt_obj += col->coef * col->pval;
         }
      }
      else if (npp->sol == GLP_MIP)
      {  /* store MIP solution */
         xassert(!npp->scaling);
         orig->mip_stat = npp->i_stat;
         orig->mip_obj = orig->c0;
         for (i = 1; i <= orig->m; i++)
         {  row = orig->row[i];
            row->mipx = npp->r_prim[i];
         }
         for (j = 1; j <= orig->n; j++)
         {  col = orig->col[j];
            col->mipx = npp->c_prim[j];
            if (col->kind == GLP_IV)
               xassert(col->mipx == floor(col->mipx));
            orig->mip_obj += col->coef * col->mipx;
         }
      }
      else
         xassert(npp != npp);
      return;
}

void npp_delete_wksp(NPP *npp)
{     /* delete LP/MIP preprocessor workspace */
      if (npp->pool != NULL)
         dmp_delete_pool(npp->pool);
      if (npp->stack != NULL)
         dmp_delete_pool(npp->stack);
      if (npp->row_ref != NULL)
         xfree(npp->row_ref);
      if (npp->col_ref != NULL)
         xfree(npp->col_ref);
      if (npp->r_stat != NULL)
         xfree(npp->r_stat);
      if (npp->r_prim != NULL)
         xfree(npp->r_prim);
      if (npp->r_dual != NULL)
         xfree(npp->r_dual);
      if (npp->c_stat != NULL)
         xfree(npp->c_stat);
      if (npp->c_prim != NULL)
         xfree(npp->c_prim);
      if (npp->c_dual != NULL)
         xfree(npp->c_dual);
      xfree(npp);
      return;
}

/* eof */
