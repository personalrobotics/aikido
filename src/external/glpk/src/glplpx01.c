/* glplpx01.c (obsolete API routines) */

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
#if 0 /* 13/IV-2009 */
#include "glpmps.h"
#endif

LPX *lpx_create_prob(void)
{     /* create problem object */
      return glp_create_prob();
}

void lpx_set_prob_name(LPX *lp, const char *name)
{     /* assign (change) problem name */
      glp_set_prob_name(lp, name);
      return;
}

void lpx_set_obj_name(LPX *lp, const char *name)
{     /* assign (change) objective function name */
      glp_set_obj_name(lp, name);
      return;
}

void lpx_set_obj_dir(LPX *lp, int dir)
{     /* set (change) optimization direction flag */
      glp_set_obj_dir(lp, dir - LPX_MIN + GLP_MIN);
      return;
}

int lpx_add_rows(LPX *lp, int nrs)
{     /* add new rows to problem object */
      return glp_add_rows(lp, nrs);
}

int lpx_add_cols(LPX *lp, int ncs)
{     /* add new columns to problem object */
      return glp_add_cols(lp, ncs);
}

void lpx_set_row_name(LPX *lp, int i, const char *name)
{     /* assign (change) row name */
      glp_set_row_name(lp, i, name);
      return;
}

void lpx_set_col_name(LPX *lp, int j, const char *name)
{     /* assign (change) column name */
      glp_set_col_name(lp, j, name);
      return;
}

void lpx_set_row_bnds(LPX *lp, int i, int type, double lb, double ub)
{     /* set (change) row bounds */
      glp_set_row_bnds(lp, i, type - LPX_FR + GLP_FR, lb, ub);
      return;
}

void lpx_set_col_bnds(LPX *lp, int j, int type, double lb, double ub)
{     /* set (change) column bounds */
      glp_set_col_bnds(lp, j, type - LPX_FR + GLP_FR, lb, ub);
      return;
}

void lpx_set_obj_coef(glp_prob *lp, int j, double coef)
{     /* set (change) obj. coefficient or constant term */
      glp_set_obj_coef(lp, j, coef);
      return;
}

void lpx_set_mat_row(LPX *lp, int i, int len, const int ind[],
      const double val[])
{     /* set (replace) row of the constraint matrix */
      glp_set_mat_row(lp, i, len, ind, val);
      return;
}

void lpx_set_mat_col(LPX *lp, int j, int len, const int ind[],
      const double val[])
{     /* set (replace) column of the constraint matrix */
      glp_set_mat_col(lp, j, len, ind, val);
      return;
}

void lpx_load_matrix(LPX *lp, int ne, const int ia[], const int ja[],
      const double ar[])
{     /* load (replace) the whole constraint matrix */
      glp_load_matrix(lp, ne, ia, ja, ar);
      return;
}

void lpx_del_rows(LPX *lp, int nrs, const int num[])
{     /* delete specified rows from problem object */
      glp_del_rows(lp, nrs, num);
      return;
}

void lpx_del_cols(LPX *lp, int ncs, const int num[])
{     /* delete specified columns from problem object */
      glp_del_cols(lp, ncs, num);
      return;
}

void lpx_delete_prob(LPX *lp)
{     /* delete problem object */
      glp_delete_prob(lp);
      return;
}

const char *lpx_get_prob_name(LPX *lp)
{     /* retrieve problem name */
      return glp_get_prob_name(lp);
}

const char *lpx_get_obj_name(LPX *lp)
{     /* retrieve objective function name */
      return glp_get_obj_name(lp);
}

int lpx_get_obj_dir(LPX *lp)
{     /* retrieve optimization direction flag */
      return glp_get_obj_dir(lp) - GLP_MIN + LPX_MIN;
}

int lpx_get_num_rows(LPX *lp)
{     /* retrieve number of rows */
      return glp_get_num_rows(lp);
}

int lpx_get_num_cols(LPX *lp)
{     /* retrieve number of columns */
      return glp_get_num_cols(lp);
}

const char *lpx_get_row_name(LPX *lp, int i)
{     /* retrieve row name */
      return glp_get_row_name(lp, i);
}

const char *lpx_get_col_name(LPX *lp, int j)
{     /* retrieve column name */
      return glp_get_col_name(lp, j);
}

int lpx_get_row_type(LPX *lp, int i)
{     /* retrieve row type */
      return glp_get_row_type(lp, i) - GLP_FR + LPX_FR;
}

double lpx_get_row_lb(glp_prob *lp, int i)
{     /* retrieve row lower bound */
      double lb;
      lb = glp_get_row_lb(lp, i);
      if (lb == -DBL_MAX) lb = 0.0;
      return lb;
}

double lpx_get_row_ub(glp_prob *lp, int i)
{     /* retrieve row upper bound */
      double ub;
      ub = glp_get_row_ub(lp, i);
      if (ub == +DBL_MAX) ub = 0.0;
      return ub;
}

void lpx_get_row_bnds(glp_prob *lp, int i, int *typx, double *lb,
      double *ub)
{     /* retrieve row bounds */
      if (typx != NULL) *typx = lpx_get_row_type(lp, i);
      if (lb != NULL) *lb = lpx_get_row_lb(lp, i);
      if (ub != NULL) *ub = lpx_get_row_ub(lp, i);
      return;
}

int lpx_get_col_type(LPX *lp, int j)
{     /* retrieve column type */
      return glp_get_col_type(lp, j) - GLP_FR + LPX_FR;
}

double lpx_get_col_lb(glp_prob *lp, int j)
{     /* retrieve column lower bound */
      double lb;
      lb = glp_get_col_lb(lp, j);
      if (lb == -DBL_MAX) lb = 0.0;
      return lb;
}

double lpx_get_col_ub(glp_prob *lp, int j)
{     /* retrieve column upper bound */
      double ub;
      ub = glp_get_col_ub(lp, j);
      if (ub == +DBL_MAX) ub = 0.0;
      return ub;
}

void lpx_get_col_bnds(glp_prob *lp, int j, int *typx, double *lb,
      double *ub)
{     /* retrieve column bounds */
      if (typx != NULL) *typx = lpx_get_col_type(lp, j);
      if (lb != NULL) *lb = lpx_get_col_lb(lp, j);
      if (ub != NULL) *ub = lpx_get_col_ub(lp, j);
      return;
}

double lpx_get_obj_coef(LPX *lp, int j)
{     /* retrieve obj. coefficient or constant term */
      return glp_get_obj_coef(lp, j);
}

int lpx_get_num_nz(LPX *lp)
{     /* retrieve number of constraint coefficients */
      return glp_get_num_nz(lp);
}

int lpx_get_mat_row(LPX *lp, int i, int ind[], double val[])
{     /* retrieve row of the constraint matrix */
      return glp_get_mat_row(lp, i, ind, val);
}

int lpx_get_mat_col(LPX *lp, int j, int ind[], double val[])
{     /* retrieve column of the constraint matrix */
      return glp_get_mat_col(lp, j, ind, val);
}

void lpx_create_index(LPX *lp)
{     /* create the name index */
      glp_create_index(lp);
      return;
}

int lpx_find_row(LPX *lp, const char *name)
{     /* find row by its name */
      return glp_find_row(lp, name);
}

int lpx_find_col(LPX *lp, const char *name)
{     /* find column by its name */
      return glp_find_col(lp, name);
}

void lpx_delete_index(LPX *lp)
{     /* delete the name index */
      glp_delete_index(lp);
      return;
}

void lpx_scale_prob(LPX *lp)
{     /* scale problem data */
      switch (lpx_get_int_parm(lp, LPX_K_SCALE))
      {  case 0:
            /* no scaling */
            glp_unscale_prob(lp);
            break;
         case 1:
            /* equilibration scaling */
            glp_scale_prob(lp, GLP_SF_EQ);
            break;
         case 2:
            /* geometric mean scaling */
            glp_scale_prob(lp, GLP_SF_GM);
            break;
         case 3:
            /* geometric mean scaling, then equilibration scaling */
            glp_scale_prob(lp, GLP_SF_GM | GLP_SF_EQ);
            break;
         default:
            xassert(lp != lp);
      }
      return;
}

void lpx_unscale_prob(LPX *lp)
{     /* unscale problem data */
      glp_unscale_prob(lp);
      return;
}

void lpx_set_row_stat(LPX *lp, int i, int stat)
{     /* set (change) row status */
      glp_set_row_stat(lp, i, stat - LPX_BS + GLP_BS);
      return;
}

void lpx_set_col_stat(LPX *lp, int j, int stat)
{     /* set (change) column status */
      glp_set_col_stat(lp, j, stat - LPX_BS + GLP_BS);
      return;
}

void lpx_std_basis(LPX *lp)
{     /* construct standard initial LP basis */
      glp_std_basis(lp);
      return;
}

void lpx_adv_basis(LPX *lp)
{     /* construct advanced initial LP basis */
      glp_adv_basis(lp, 0);
      return;
}

void lpx_cpx_basis(LPX *lp)
{     /* construct Bixby's initial LP basis */
      glp_cpx_basis(lp);
      return;
}

static void fill_smcp(LPX *lp, glp_smcp *parm)
{     glp_init_smcp(parm);
      switch (lpx_get_int_parm(lp, LPX_K_MSGLEV))
      {  case 0:  parm->msg_lev = GLP_MSG_OFF;   break;
         case 1:  parm->msg_lev = GLP_MSG_ERR;   break;
         case 2:  parm->msg_lev = GLP_MSG_ON;    break;
         case 3:  parm->msg_lev = GLP_MSG_ALL;   break;
         default: xassert(lp != lp);
      }
      switch (lpx_get_int_parm(lp, LPX_K_DUAL))
      {  case 0:  parm->meth = GLP_PRIMAL;       break;
         case 1:  parm->meth = GLP_DUAL;         break;
         default: xassert(lp != lp);
      }
      switch (lpx_get_int_parm(lp, LPX_K_PRICE))
      {  case 0:  parm->pricing = GLP_PT_STD;    break;
         case 1:  parm->pricing = GLP_PT_PSE;    break;
         default: xassert(lp != lp);
      }
      if (lpx_get_real_parm(lp, LPX_K_RELAX) == 0.0)
         parm->r_test = GLP_RT_STD;
      else
         parm->r_test = GLP_RT_HAR;
      parm->tol_bnd = lpx_get_real_parm(lp, LPX_K_TOLBND);
      parm->tol_dj  = lpx_get_real_parm(lp, LPX_K_TOLDJ);
      parm->tol_piv = lpx_get_real_parm(lp, LPX_K_TOLPIV);
      parm->obj_ll  = lpx_get_real_parm(lp, LPX_K_OBJLL);
      parm->obj_ul  = lpx_get_real_parm(lp, LPX_K_OBJUL);
      if (lpx_get_int_parm(lp, LPX_K_ITLIM) < 0)
         parm->it_lim = INT_MAX;
      else
         parm->it_lim = lpx_get_int_parm(lp, LPX_K_ITLIM);
      if (lpx_get_real_parm(lp, LPX_K_TMLIM) < 0.0)
         parm->tm_lim = INT_MAX;
      else
         parm->tm_lim =
            (int)(1000.0 * lpx_get_real_parm(lp, LPX_K_TMLIM));
      parm->out_frq = lpx_get_int_parm(lp, LPX_K_OUTFRQ);
      parm->out_dly =
            (int)(1000.0 * lpx_get_real_parm(lp, LPX_K_OUTDLY));
      switch (lpx_get_int_parm(lp, LPX_K_PRESOL))
      {  case 0:  parm->presolve = GLP_OFF;      break;
         case 1:  parm->presolve = GLP_ON;       break;
         default: xassert(lp != lp);
      }
      return;
}

int lpx_simplex(LPX *lp)
{     /* easy-to-use driver to the simplex method */
      glp_smcp parm;
      int ret;
      fill_smcp(lp, &parm);
      ret = glp_simplex(lp, &parm);
      switch (ret)
      {  case 0:           ret = LPX_E_OK;      break;
         case GLP_EBADB:
         case GLP_ESING:
         case GLP_ECOND:
         case GLP_EBOUND:  ret = LPX_E_FAULT;   break;
         case GLP_EFAIL:   ret = LPX_E_SING;    break;
         case GLP_EOBJLL:  ret = LPX_E_OBJLL;   break;
         case GLP_EOBJUL:  ret = LPX_E_OBJUL;   break;
         case GLP_EITLIM:  ret = LPX_E_ITLIM;   break;
         case GLP_ETMLIM:  ret = LPX_E_TMLIM;   break;
         case GLP_ENOPFS:  ret = LPX_E_NOPFS;   break;
         case GLP_ENODFS:  ret = LPX_E_NODFS;   break;
         default:          xassert(ret != ret);
      }
      return ret;
}

int lpx_exact(LPX *lp)
{     /* easy-to-use driver to the exact simplex method */
      glp_smcp parm;
      int ret;
      fill_smcp(lp, &parm);
      ret = glp_exact(lp, &parm);
      switch (ret)
      {  case 0:           ret = LPX_E_OK;      break;
         case GLP_EBADB:
         case GLP_ESING:
         case GLP_EBOUND:
         case GLP_EFAIL:   ret = LPX_E_FAULT;   break;
         case GLP_EITLIM:  ret = LPX_E_ITLIM;   break;
         case GLP_ETMLIM:  ret = LPX_E_TMLIM;   break;
         default:          xassert(ret != ret);
      }
      return ret;
}

int lpx_get_status(glp_prob *lp)
{     /* retrieve generic status of basic solution */
      int status;
      switch (glp_get_status(lp))
      {  case GLP_OPT:    status = LPX_OPT;    break;
         case GLP_FEAS:   status = LPX_FEAS;   break;
         case GLP_INFEAS: status = LPX_INFEAS; break;
         case GLP_NOFEAS: status = LPX_NOFEAS; break;
         case GLP_UNBND:  status = LPX_UNBND;  break;
         case GLP_UNDEF:  status = LPX_UNDEF;  break;
         default:         xassert(lp != lp);
      }
      return status;
}

int lpx_get_prim_stat(glp_prob *lp)
{     /* retrieve status of primal basic solution */
      return glp_get_prim_stat(lp) - GLP_UNDEF + LPX_P_UNDEF;
}

int lpx_get_dual_stat(glp_prob *lp)
{     /* retrieve status of dual basic solution */
      return glp_get_dual_stat(lp) - GLP_UNDEF + LPX_D_UNDEF;
}

double lpx_get_obj_val(LPX *lp)
{     /* retrieve objective value (basic solution) */
      return glp_get_obj_val(lp);
}

int lpx_get_row_stat(LPX *lp, int i)
{     /* retrieve row status (basic solution) */
      return glp_get_row_stat(lp, i) - GLP_BS + LPX_BS;
}

double lpx_get_row_prim(LPX *lp, int i)
{     /* retrieve row primal value (basic solution) */
      return glp_get_row_prim(lp, i);
}

double lpx_get_row_dual(LPX *lp, int i)
{     /* retrieve row dual value (basic solution) */
      return glp_get_row_dual(lp, i);
}

void lpx_get_row_info(glp_prob *lp, int i, int *tagx, double *vx,
      double *dx)
{     /* obtain row solution information */
      if (tagx != NULL) *tagx = lpx_get_row_stat(lp, i);
      if (vx != NULL) *vx = lpx_get_row_prim(lp, i);
      if (dx != NULL) *dx = lpx_get_row_dual(lp, i);
      return;
}

int lpx_get_col_stat(LPX *lp, int j)
{     /* retrieve column status (basic solution) */
      return glp_get_col_stat(lp, j) - GLP_BS + LPX_BS;
}

double lpx_get_col_prim(LPX *lp, int j)
{     /* retrieve column primal value (basic solution) */
      return glp_get_col_prim(lp, j);
}

double lpx_get_col_dual(glp_prob *lp, int j)
{     /* retrieve column dual value (basic solution) */
      return glp_get_col_dual(lp, j);
}

void lpx_get_col_info(glp_prob *lp, int j, int *tagx, double *vx,
      double *dx)
{     /* obtain column solution information */
      if (tagx != NULL) *tagx = lpx_get_col_stat(lp, j);
      if (vx != NULL) *vx = lpx_get_col_prim(lp, j);
      if (dx != NULL) *dx = lpx_get_col_dual(lp, j);
      return;
}

int lpx_get_ray_info(LPX *lp)
{     /* determine what causes primal unboundness */
      return glp_get_unbnd_ray(lp);
}

void lpx_check_kkt(LPX *lp, int scaled, LPXKKT *kkt)
{     /* check Karush-Kuhn-Tucker conditions */
      int ae_ind, re_ind;
      double ae_max, re_max;
      xassert(scaled == scaled);
      _glp_check_kkt(lp, GLP_SOL, GLP_KKT_PE, &ae_max, &ae_ind, &re_max,
         &re_ind);
      kkt->pe_ae_max = ae_max;
      kkt->pe_ae_row = ae_ind;
      kkt->pe_re_max = re_max;
      kkt->pe_re_row = re_ind;
      if (re_max <= 1e-9)
         kkt->pe_quality = 'H';
      else if (re_max <= 1e-6)
         kkt->pe_quality = 'M';
      else if (re_max <= 1e-3)
         kkt->pe_quality = 'L';
      else
         kkt->pe_quality = '?';
      _glp_check_kkt(lp, GLP_SOL, GLP_KKT_PB, &ae_max, &ae_ind, &re_max,
         &re_ind);
      kkt->pb_ae_max = ae_max;
      kkt->pb_ae_ind = ae_ind;
      kkt->pb_re_max = re_max;
      kkt->pb_re_ind = re_ind;
      if (re_max <= 1e-9)
         kkt->pb_quality = 'H';
      else if (re_max <= 1e-6)
         kkt->pb_quality = 'M';
      else if (re_max <= 1e-3)
         kkt->pb_quality = 'L';
      else
         kkt->pb_quality = '?';
      _glp_check_kkt(lp, GLP_SOL, GLP_KKT_DE, &ae_max, &ae_ind, &re_max,
         &re_ind);
      kkt->de_ae_max = ae_max;
      if (ae_ind == 0)
         kkt->de_ae_col = 0;
      else
         kkt->de_ae_col = ae_ind - lp->m;
      kkt->de_re_max = re_max;
      if (re_ind == 0)
         kkt->de_re_col = 0;
      else
         kkt->de_re_col = ae_ind - lp->m;
      if (re_max <= 1e-9)
         kkt->de_quality = 'H';
      else if (re_max <= 1e-6)
         kkt->de_quality = 'M';
      else if (re_max <= 1e-3)
         kkt->de_quality = 'L';
      else
         kkt->de_quality = '?';
      _glp_check_kkt(lp, GLP_SOL, GLP_KKT_DB, &ae_max, &ae_ind, &re_max,
         &re_ind);
      kkt->db_ae_max = ae_max;
      kkt->db_ae_ind = ae_ind;
      kkt->db_re_max = re_max;
      kkt->db_re_ind = re_ind;
      if (re_max <= 1e-9)
         kkt->db_quality = 'H';
      else if (re_max <= 1e-6)
         kkt->db_quality = 'M';
      else if (re_max <= 1e-3)
         kkt->db_quality = 'L';
      else
         kkt->db_quality = '?';
      kkt->cs_ae_max = 0.0, kkt->cs_ae_ind = 0;
      kkt->cs_re_max = 0.0, kkt->cs_re_ind = 0;
      kkt->cs_quality = 'H';
      return;
}

int lpx_warm_up(LPX *lp)
{     /* "warm up" LP basis */
      int ret;
      ret = glp_warm_up(lp);
      if (ret == 0)
         ret = LPX_E_OK;
      else if (ret == GLP_EBADB)
         ret = LPX_E_BADB;
      else if (ret == GLP_ESING)
         ret = LPX_E_SING;
      else if (ret == GLP_ECOND)
         ret = LPX_E_SING;
      else
         xassert(ret != ret);
      return ret;
}

int lpx_eval_tab_row(LPX *lp, int k, int ind[], double val[])
{     /* compute row of the simplex tableau */
      return glp_eval_tab_row(lp, k, ind, val);
}

int lpx_eval_tab_col(LPX *lp, int k, int ind[], double val[])
{     /* compute column of the simplex tableau */
      return glp_eval_tab_col(lp, k, ind, val);
}

int lpx_interior(LPX *lp)
{     /* easy-to-use driver to the interior-point method */
      int ret;
      ret = glp_interior(lp, NULL);
      switch (ret)
      {  case 0:           ret = LPX_E_OK;      break;
         case GLP_EFAIL:   ret = LPX_E_FAULT;   break;
         case GLP_ENOFEAS: ret = LPX_E_NOFEAS;  break;
         case GLP_ENOCVG:  ret = LPX_E_NOCONV;  break;
         case GLP_EITLIM:  ret = LPX_E_ITLIM;   break;
         case GLP_EINSTAB: ret = LPX_E_INSTAB;  break;
         default:          xassert(ret != ret);
      }
      return ret;
}

int lpx_ipt_status(glp_prob *lp)
{     /* retrieve status of interior-point solution */
      int status;
      switch (glp_ipt_status(lp))
      {  case GLP_UNDEF:  status = LPX_T_UNDEF;  break;
         case GLP_OPT:    status = LPX_T_OPT;    break;
         default:         xassert(lp != lp);
      }
      return status;
}

double lpx_ipt_obj_val(LPX *lp)
{     /* retrieve objective value (interior point) */
      return glp_ipt_obj_val(lp);
}

double lpx_ipt_row_prim(LPX *lp, int i)
{     /* retrieve row primal value (interior point) */
      return glp_ipt_row_prim(lp, i);
}

double lpx_ipt_row_dual(LPX *lp, int i)
{     /* retrieve row dual value (interior point) */
      return glp_ipt_row_dual(lp, i);
}

double lpx_ipt_col_prim(LPX *lp, int j)
{     /* retrieve column primal value (interior point) */
      return glp_ipt_col_prim(lp, j);
}

double lpx_ipt_col_dual(LPX *lp, int j)
{     /* retrieve column dual value (interior point) */
      return glp_ipt_col_dual(lp, j);
}

void lpx_set_class(LPX *lp, int klass)
{     /* set problem class */
      xassert(lp == lp);
      if (!(klass == LPX_LP || klass == LPX_MIP))
         xerror("lpx_set_class: invalid problem class\n");
      return;
}

int lpx_get_class(LPX *lp)
{     /* determine problem klass */
      return glp_get_num_int(lp) == 0 ? LPX_LP : LPX_MIP;
}

void lpx_set_col_kind(LPX *lp, int j, int kind)
{     /* set (change) column kind */
      glp_set_col_kind(lp, j, kind - LPX_CV + GLP_CV);
      return;
}

int lpx_get_col_kind(LPX *lp, int j)
{     /* retrieve column kind */
      return glp_get_col_kind(lp, j) == GLP_CV ? LPX_CV : LPX_IV;
}

int lpx_get_num_int(LPX *lp)
{     /* retrieve number of integer columns */
      return glp_get_num_int(lp);
}

int lpx_get_num_bin(LPX *lp)
{     /* retrieve number of binary columns */
      return glp_get_num_bin(lp);
}

static int solve_mip(LPX *lp, int presolve)
{     glp_iocp parm;
      int ret;
      glp_init_iocp(&parm);
      switch (lpx_get_int_parm(lp, LPX_K_MSGLEV))
      {  case 0:  parm.msg_lev = GLP_MSG_OFF;   break;
         case 1:  parm.msg_lev = GLP_MSG_ERR;   break;
         case 2:  parm.msg_lev = GLP_MSG_ON;    break;
         case 3:  parm.msg_lev = GLP_MSG_ALL;   break;
         default: xassert(lp != lp);
      }
      switch (lpx_get_int_parm(lp, LPX_K_BRANCH))
      {  case 0:  parm.br_tech = GLP_BR_FFV;    break;
         case 1:  parm.br_tech = GLP_BR_LFV;    break;
         case 2:  parm.br_tech = GLP_BR_DTH;    break;
         case 3:  parm.br_tech = GLP_BR_MFV;    break;
         default: xassert(lp != lp);
      }
      switch (lpx_get_int_parm(lp, LPX_K_BTRACK))
      {  case 0:  parm.bt_tech = GLP_BT_DFS;    break;
         case 1:  parm.bt_tech = GLP_BT_BFS;    break;
         case 2:  parm.bt_tech = GLP_BT_BPH;    break;
         case 3:  parm.bt_tech = GLP_BT_BLB;    break;
         default: xassert(lp != lp);
      }
      parm.tol_int = lpx_get_real_parm(lp, LPX_K_TOLINT);
      parm.tol_obj = lpx_get_real_parm(lp, LPX_K_TOLOBJ);
      if (lpx_get_real_parm(lp, LPX_K_TMLIM) < 0.0 ||
          lpx_get_real_parm(lp, LPX_K_TMLIM) > 1e6)
         parm.tm_lim = INT_MAX;
      else
         parm.tm_lim =
            (int)(1000.0 * lpx_get_real_parm(lp, LPX_K_TMLIM));
      parm.mip_gap = lpx_get_real_parm(lp, LPX_K_MIPGAP);
      if (lpx_get_int_parm(lp, LPX_K_USECUTS) & LPX_C_GOMORY)
         parm.gmi_cuts = GLP_ON;
      else
         parm.gmi_cuts = GLP_OFF;
      if (lpx_get_int_parm(lp, LPX_K_USECUTS) & LPX_C_MIR)
         parm.mir_cuts = GLP_ON;
      else
         parm.mir_cuts = GLP_OFF;
      if (lpx_get_int_parm(lp, LPX_K_USECUTS) & LPX_C_COVER)
         parm.cov_cuts = GLP_ON;
      else
         parm.cov_cuts = GLP_OFF;
      if (lpx_get_int_parm(lp, LPX_K_USECUTS) & LPX_C_CLIQUE)
         parm.clq_cuts = GLP_ON;
      else
         parm.clq_cuts = GLP_OFF;
      parm.presolve = presolve;
      if (lpx_get_int_parm(lp, LPX_K_BINARIZE))
         parm.binarize = GLP_ON;
      ret = glp_intopt(lp, &parm);
      switch (ret)
      {  case 0:           ret = LPX_E_OK;      break;
         case GLP_ENOPFS:  ret = LPX_E_NOPFS;   break;
         case GLP_ENODFS:  ret = LPX_E_NODFS;   break;
         case GLP_EBOUND:
         case GLP_EROOT:   ret = LPX_E_FAULT;   break;
         case GLP_EFAIL:   ret = LPX_E_SING;    break;
         case GLP_EMIPGAP: ret = LPX_E_MIPGAP;  break;
         case GLP_ETMLIM:  ret = LPX_E_TMLIM;   break;
         default:          xassert(ret != ret);
      }
      return ret;
}

int lpx_integer(LPX *lp)
{     /* easy-to-use driver to the branch-and-bound method */
      return solve_mip(lp, GLP_OFF);
}

int lpx_intopt(LPX *lp)
{     /* easy-to-use driver to the branch-and-bound method */
      return solve_mip(lp, GLP_ON);
}

int lpx_mip_status(glp_prob *lp)
{     /* retrieve status of MIP solution */
      int status;
      switch (glp_mip_status(lp))
      {  case GLP_UNDEF:  status = LPX_I_UNDEF;  break;
         case GLP_OPT:    status = LPX_I_OPT;    break;
         case GLP_FEAS:   status = LPX_I_FEAS;   break;
         case GLP_NOFEAS: status = LPX_I_NOFEAS; break;
         default:         xassert(lp != lp);
      }
      return status;
}

double lpx_mip_obj_val(LPX *lp)
{     /* retrieve objective value (MIP solution) */
      return glp_mip_obj_val(lp);
}

double lpx_mip_row_val(LPX *lp, int i)
{     /* retrieve row value (MIP solution) */
      return glp_mip_row_val(lp, i);
}

double lpx_mip_col_val(LPX *lp, int j)
{     /* retrieve column value (MIP solution) */
      return glp_mip_col_val(lp, j);
}

void lpx_check_int(LPX *lp, LPXKKT *kkt)
{     /* check integer feasibility conditions */
      int ae_ind, re_ind;
      double ae_max, re_max;
      _glp_check_kkt(lp, GLP_MIP, GLP_KKT_PE, &ae_max, &ae_ind, &re_max,
         &re_ind);
      kkt->pe_ae_max = ae_max;
      kkt->pe_ae_row = ae_ind;
      kkt->pe_re_max = re_max;
      kkt->pe_re_row = re_ind;
      if (re_max <= 1e-9)
         kkt->pe_quality = 'H';
      else if (re_max <= 1e-6)
         kkt->pe_quality = 'M';
      else if (re_max <= 1e-3)
         kkt->pe_quality = 'L';
      else
         kkt->pe_quality = '?';
      _glp_check_kkt(lp, GLP_MIP, GLP_KKT_PB, &ae_max, &ae_ind, &re_max,
         &re_ind);
      kkt->pb_ae_max = ae_max;
      kkt->pb_ae_ind = ae_ind;
      kkt->pb_re_max = re_max;
      kkt->pb_re_ind = re_ind;
      if (re_max <= 1e-9)
         kkt->pb_quality = 'H';
      else if (re_max <= 1e-6)
         kkt->pb_quality = 'M';
      else if (re_max <= 1e-3)
         kkt->pb_quality = 'L';
      else
         kkt->pb_quality = '?';
      return;
}

void lpx_reset_parms(LPX *lp)
{     /* reset control parameters to default values */
      struct LPXCPS *cps = lp->cps;
      cps->msg_lev  = 3;
      cps->scale    = 1;
      cps->dual     = 0;
      cps->price    = 1;
      cps->relax    = 0.07;
      cps->tol_bnd  = 1e-7;
      cps->tol_dj   = 1e-7;
      cps->tol_piv  = 1e-9;
      cps->round    = 0;
      cps->obj_ll   = -DBL_MAX;
      cps->obj_ul   = +DBL_MAX;
      cps->it_lim   = -1;
      lp->it_cnt   = 0;
      cps->tm_lim   = -1.0;
      cps->out_frq  = 200;
      cps->out_dly  = 0.0;
      cps->branch   = 2;
      cps->btrack   = 3;
      cps->tol_int  = 1e-5;
      cps->tol_obj  = 1e-7;
      cps->mps_info = 1;
      cps->mps_obj  = 2;
      cps->mps_orig = 0;
      cps->mps_wide = 1;
      cps->mps_free = 0;
      cps->mps_skip = 0;
      cps->lpt_orig = 0;
      cps->presol = 0;
      cps->binarize = 0;
      cps->use_cuts = 0;
      cps->mip_gap = 0.0;
      return;
}

void lpx_set_int_parm(LPX *lp, int parm, int val)
{     /* set (change) integer control parameter */
      struct LPXCPS *cps = lp->cps;
      switch (parm)
      {  case LPX_K_MSGLEV:
            if (!(0 <= val && val <= 3))
               xerror("lpx_set_int_parm: MSGLEV = %d; invalid value\n",
                  val);
            cps->msg_lev = val;
            break;
         case LPX_K_SCALE:
            if (!(0 <= val && val <= 3))
               xerror("lpx_set_int_parm: SCALE = %d; invalid value\n",
                  val);
            cps->scale = val;
            break;
         case LPX_K_DUAL:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: DUAL = %d; invalid value\n",
                  val);
            cps->dual = val;
            break;
         case LPX_K_PRICE:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: PRICE = %d; invalid value\n",
                  val);
            cps->price = val;
            break;
         case LPX_K_ROUND:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: ROUND = %d; invalid value\n",
                  val);
            cps->round = val;
            break;
         case LPX_K_ITLIM:
            cps->it_lim = val;
            break;
         case LPX_K_ITCNT:
            lp->it_cnt = val;
            break;
         case LPX_K_OUTFRQ:
            if (!(val > 0))
               xerror("lpx_set_int_parm: OUTFRQ = %d; invalid value\n",
                  val);
            cps->out_frq = val;
            break;
         case LPX_K_BRANCH:
            if (!(val == 0 || val == 1 || val == 2 || val == 3))
               xerror("lpx_set_int_parm: BRANCH = %d; invalid value\n",
                  val);
            cps->branch = val;
            break;
         case LPX_K_BTRACK:
            if (!(val == 0 || val == 1 || val == 2 || val == 3))
               xerror("lpx_set_int_parm: BTRACK = %d; invalid value\n",
                  val);
            cps->btrack = val;
            break;
         case LPX_K_MPSINFO:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: MPSINFO = %d; invalid value\n",
                  val);
            cps->mps_info = val;
            break;
         case LPX_K_MPSOBJ:
            if (!(val == 0 || val == 1 || val == 2))
               xerror("lpx_set_int_parm: MPSOBJ = %d; invalid value\n",
                  val);
            cps->mps_obj = val;
            break;
         case LPX_K_MPSORIG:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: MPSORIG = %d; invalid value\n",
                  val);
            cps->mps_orig = val;
            break;
         case LPX_K_MPSWIDE:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: MPSWIDE = %d; invalid value\n",
                  val);
            cps->mps_wide = val;
            break;
         case LPX_K_MPSFREE:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: MPSFREE = %d; invalid value\n",
                  val);
            cps->mps_free = val;
            break;
         case LPX_K_MPSSKIP:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: MPSSKIP = %d; invalid value\n",
                  val);
            cps->mps_skip = val;
            break;
         case LPX_K_LPTORIG:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: LPTORIG = %d; invalid value\n",
                  val);
            cps->lpt_orig = val;
            break;
         case LPX_K_PRESOL:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: PRESOL = %d; invalid value\n",
                  val);
            cps->presol = val;
            break;
         case LPX_K_BINARIZE:
            if (!(val == 0 || val == 1))
               xerror("lpx_set_int_parm: BINARIZE = %d; invalid value\n"
                  , val);
            cps->binarize = val;
            break;
         case LPX_K_USECUTS:
            if (val & ~LPX_C_ALL)
            xerror("lpx_set_int_parm: USECUTS = 0x%X; invalid value\n",
                  val);
            cps->use_cuts = val;
            break;
         case LPX_K_BFTYPE:
#if 0
            if (!(1 <= val && val <= 3))
               xerror("lpx_set_int_parm: BFTYPE = %d; invalid value\n",
                  val);
            cps->bf_type = val;
#else
            {  glp_bfcp parm;
               glp_get_bfcp(lp, &parm);
               switch (val)
               {  case 1:
                     parm.type = GLP_BF_FT; break;
                  case 2:
                     parm.type = GLP_BF_BG; break;
                  case 3:
                     parm.type = GLP_BF_GR; break;
                  default:
                     xerror("lpx_set_int_parm: BFTYPE = %d; invalid val"
                        "ue\n", val);
               }
               glp_set_bfcp(lp, &parm);
            }
#endif
            break;
         default:
            xerror("lpx_set_int_parm: parm = %d; invalid parameter\n",
               parm);
      }
      return;
}

int lpx_get_int_parm(LPX *lp, int parm)
{     /* query integer control parameter */
      struct LPXCPS *cps = lp->cps;
      int val = 0;
      switch (parm)
      {  case LPX_K_MSGLEV:
            val = cps->msg_lev; break;
         case LPX_K_SCALE:
            val = cps->scale; break;
         case LPX_K_DUAL:
            val = cps->dual; break;
         case LPX_K_PRICE:
            val = cps->price; break;
         case LPX_K_ROUND:
            val = cps->round; break;
         case LPX_K_ITLIM:
            val = cps->it_lim; break;
         case LPX_K_ITCNT:
            val = lp->it_cnt; break;
         case LPX_K_OUTFRQ:
            val = cps->out_frq; break;
         case LPX_K_BRANCH:
            val = cps->branch; break;
         case LPX_K_BTRACK:
            val = cps->btrack; break;
         case LPX_K_MPSINFO:
            val = cps->mps_info; break;
         case LPX_K_MPSOBJ:
            val = cps->mps_obj; break;
         case LPX_K_MPSORIG:
            val = cps->mps_orig; break;
         case LPX_K_MPSWIDE:
            val = cps->mps_wide; break;
         case LPX_K_MPSFREE:
            val = cps->mps_free; break;
         case LPX_K_MPSSKIP:
            val = cps->mps_skip; break;
         case LPX_K_LPTORIG:
            val = cps->lpt_orig; break;
         case LPX_K_PRESOL:
            val = cps->presol; break;
         case LPX_K_BINARIZE:
            val = cps->binarize; break;
         case LPX_K_USECUTS:
            val = cps->use_cuts; break;
         case LPX_K_BFTYPE:
#if 0
            val = cps->bf_type; break;
#else
            {  glp_bfcp parm;
               glp_get_bfcp(lp, &parm);
               switch (parm.type)
               {  case GLP_BF_FT:
                     val = 1; break;
                  case GLP_BF_BG:
                     val = 2; break;
                  case GLP_BF_GR:
                     val = 3; break;
                  default:
                     xassert(lp != lp);
               }
            }
            break;
#endif
         default:
            xerror("lpx_get_int_parm: parm = %d; invalid parameter\n",
               parm);
      }
      return val;
}

void lpx_set_real_parm(LPX *lp, int parm, double val)
{     /* set (change) real control parameter */
      struct LPXCPS *cps = lp->cps;
      switch (parm)
      {  case LPX_K_RELAX:
            if (!(0.0 <= val && val <= 1.0))
               xerror("lpx_set_real_parm: RELAX = %g; invalid value\n",
                  val);
            cps->relax = val;
            break;
         case LPX_K_TOLBND:
            if (!(DBL_EPSILON <= val && val <= 0.001))
               xerror("lpx_set_real_parm: TOLBND = %g; invalid value\n",
                  val);
#if 0
            if (cps->tol_bnd > val)
            {  /* invalidate the basic solution */
               lp->p_stat = LPX_P_UNDEF;
               lp->d_stat = LPX_D_UNDEF;
            }
#endif
            cps->tol_bnd = val;
            break;
         case LPX_K_TOLDJ:
            if (!(DBL_EPSILON <= val && val <= 0.001))
               xerror("lpx_set_real_parm: TOLDJ = %g; invalid value\n",
                  val);
#if 0
            if (cps->tol_dj > val)
            {  /* invalidate the basic solution */
               lp->p_stat = LPX_P_UNDEF;
               lp->d_stat = LPX_D_UNDEF;
            }
#endif
            cps->tol_dj = val;
            break;
         case LPX_K_TOLPIV:
            if (!(DBL_EPSILON <= val && val <= 0.001))
               xerror("lpx_set_real_parm: TOLPIV = %g; invalid value\n",
                  val);
            cps->tol_piv = val;
            break;
         case LPX_K_OBJLL:
            cps->obj_ll = val;
            break;
         case LPX_K_OBJUL:
            cps->obj_ul = val;
            break;
         case LPX_K_TMLIM:
            cps->tm_lim = val;
            break;
         case LPX_K_OUTDLY:
            cps->out_dly = val;
            break;
         case LPX_K_TOLINT:
            if (!(DBL_EPSILON <= val && val <= 0.001))
               xerror("lpx_set_real_parm: TOLINT = %g; invalid value\n",
                  val);
            cps->tol_int = val;
            break;
         case LPX_K_TOLOBJ:
            if (!(DBL_EPSILON <= val && val <= 0.001))
               xerror("lpx_set_real_parm: TOLOBJ = %g; invalid value\n",
                  val);
            cps->tol_obj = val;
            break;
         case LPX_K_MIPGAP:
            if (val < 0.0)
               xerror("lpx_set_real_parm: MIPGAP = %g; invalid value\n",
                  val);
            cps->mip_gap = val;
            break;
         default:
            xerror("lpx_set_real_parm: parm = %d; invalid parameter\n",
               parm);
      }
      return;
}

double lpx_get_real_parm(LPX *lp, int parm)
{     /* query real control parameter */
      struct LPXCPS *cps = lp->cps;
      double val = 0.0;
      switch (parm)
      {  case LPX_K_RELAX:
            val = cps->relax;
            break;
         case LPX_K_TOLBND:
            val = cps->tol_bnd;
            break;
         case LPX_K_TOLDJ:
            val = cps->tol_dj;
            break;
         case LPX_K_TOLPIV:
            val = cps->tol_piv;
            break;
         case LPX_K_OBJLL:
            val = cps->obj_ll;
            break;
         case LPX_K_OBJUL:
            val = cps->obj_ul;
            break;
         case LPX_K_TMLIM:
            val = cps->tm_lim;
            break;
         case LPX_K_OUTDLY:
            val = cps->out_dly;
            break;
         case LPX_K_TOLINT:
            val = cps->tol_int;
            break;
         case LPX_K_TOLOBJ:
            val = cps->tol_obj;
            break;
         case LPX_K_MIPGAP:
            val = cps->mip_gap;
            break;
         default:
            xerror("lpx_get_real_parm: parm = %d; invalid parameter\n",
               parm);
      }
      return val;
}

LPX *lpx_read_mps(const char *fname)
{     /* read problem data in fixed MPS format */
      LPX *lp = lpx_create_prob();
      if (glp_read_mps(lp, GLP_MPS_DECK, NULL, fname))
         lpx_delete_prob(lp), lp = NULL;
      return lp;
}

int lpx_write_mps(LPX *lp, const char *fname)
{     /* write problem data in fixed MPS format */
      return glp_write_mps(lp, GLP_MPS_DECK, NULL, fname);
}

int lpx_read_bas(LPX *lp, const char *fname)
{     /* read LP basis in fixed MPS format */
#if 0 /* 13/IV-2009 */
      return read_bas(lp, fname);
#else
      xassert(lp == lp);
      xassert(fname == fname);
      xerror("lpx_read_bas: operation not supported\n");
      return 0;
#endif
}

int lpx_write_bas(LPX *lp, const char *fname)
{     /* write LP basis in fixed MPS format */
#if 0 /* 13/IV-2009 */
      return write_bas(lp, fname);
#else
      xassert(lp == lp);
      xassert(fname == fname);
      xerror("lpx_write_bas: operation not supported\n");
      return 0;
#endif
}

LPX *lpx_read_freemps(const char *fname)
{     /* read problem data in free MPS format */
      LPX *lp = lpx_create_prob();
      if (glp_read_mps(lp, GLP_MPS_FILE, NULL, fname))
         lpx_delete_prob(lp), lp = NULL;
      return lp;
}

int lpx_write_freemps(LPX *lp, const char *fname)
{     /* write problem data in free MPS format */
      return glp_write_mps(lp, GLP_MPS_FILE, NULL, fname);
}

LPX *lpx_read_cpxlp(const char *fname)
{     /* read problem data in CPLEX LP format */
      LPX *lp;
      lp = lpx_create_prob();
      if (glp_read_lp(lp, NULL, fname))
         lpx_delete_prob(lp), lp = NULL;
      return lp;
}

int lpx_write_cpxlp(LPX *lp, const char *fname)
{     /* write problem data in CPLEX LP format */
      return glp_write_lp(lp, NULL, fname);
}

LPX *lpx_read_model(const char *model, const char *data, const char
      *output)
{     /* read LP/MIP model written in GNU MathProg language */
      LPX *lp = NULL;
      glp_tran *tran;
      /* allocate the translator workspace */
      tran = glp_mpl_alloc_wksp();
      /* read model section and optional data section */
      if (glp_mpl_read_model(tran, model, data != NULL)) goto done;
      /* read separate data section, if required */
      if (data != NULL)
         if (glp_mpl_read_data(tran, data)) goto done;
      /* generate the model */
      if (glp_mpl_generate(tran, output)) goto done;
      /* build the problem instance from the model */
      lp = glp_create_prob();
      glp_mpl_build_prob(tran, lp);
done: /* free the translator workspace */
      glp_mpl_free_wksp(tran);
      /* bring the problem object to the calling program */
      return lp;
}

int lpx_print_prob(LPX *lp, const char *fname)
{     /* write problem data in plain text format */
      return glp_write_lp(lp, NULL, fname);
}

int lpx_print_sol(LPX *lp, const char *fname)
{     /* write LP problem solution in printable format */
      return glp_print_sol(lp, fname);
}

int lpx_print_ips(LPX *lp, const char *fname)
{     /* write interior point solution in printable format */
      return glp_print_ipt(lp, fname);
}

int lpx_print_mip(LPX *lp, const char *fname)
{     /* write MIP problem solution in printable format */
      return glp_print_mip(lp, fname);
}

int lpx_is_b_avail(glp_prob *lp)
{     /* check if LP basis is available */
      return glp_bf_exists(lp);
}

int lpx_main(int argc, const char *argv[])
{     /* stand-alone LP/MIP solver */
      return glp_main(argc, argv);
}

/* eof */
