/* glpapi06.c (simplex method routines) */

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

#if 1
#define GLP_USE_NPP
#endif

#include "glpapi.h"
#ifdef GLP_USE_NPP
#include "glpnpp.h"
#else
#include "glplpp.h"
#endif
#include "glpspx.h"

/***********************************************************************
*  NAME
*
*  glp_simplex - solve LP problem with the simplex method
*
*  SYNOPSIS
*
*  int glp_simplex(glp_prob *lp, const glp_smcp *parm);
*
*  DESCRIPTION
*
*  The routine glp_simplex is a driver to the LP solver based on the
*  simplex method. This routine retrieves problem data from the
*  specified problem object, calls the solver to solve the problem
*  instance, and stores results of computations back into the problem
*  object.
*
*  The simplex solver has a set of control parameters. Values of the
*  control parameters can be passed in a structure glp_smcp, which the
*  parameter parm points to.
*
*  The parameter parm can be specified as NULL, in which case the LP
*  solver uses default settings.
*
*  RETURNS
*
*  0  The LP problem instance has been successfully solved. This code
*     does not necessarily mean that the solver has found optimal
*     solution. It only means that the solution process was successful.
*
*  GLP_EBADB
*     Unable to start the search, because the initial basis specified
*     in the problem object is invalid--the number of basic (auxiliary
*     and structural) variables is not the same as the number of rows in
*     the problem object.
*
*  GLP_ESING
*     Unable to start the search, because the basis matrix correspodning
*     to the initial basis is singular within the working precision.
*
*  GLP_ECOND
*     Unable to start the search, because the basis matrix correspodning
*     to the initial basis is ill-conditioned, i.e. its condition number
*     is too large.
*
*  GLP_EBOUND
*     Unable to start the search, because some double-bounded variables
*     have incorrect bounds.
*
*  GLP_EFAIL
*     The search was prematurely terminated due to the solver failure.
*
*  GLP_EOBJLL
*     The search was prematurely terminated, because the objective
*     function being maximized has reached its lower limit and continues
*     decreasing (dual simplex only).
*
*  GLP_EOBJUL
*     The search was prematurely terminated, because the objective
*     function being minimized has reached its upper limit and continues
*     increasing (dual simplex only).
*
*  GLP_EITLIM
*     The search was prematurely terminated, because the simplex
*     iteration limit has been exceeded.
*
*  GLP_ETMLIM
*     The search was prematurely terminated, because the time limit has
*     been exceeded.
*
*  GLP_ENOPFS
*     The LP problem instance has no primal feasible solution (only if
*     the LP presolver is used).
*
*  GLP_ENODFS
*     The LP problem instance has no dual feasible solution (only if the
*     LP presolver is used). */

static void trivial1(glp_prob *lp, const glp_smcp *parm)
{     /* solve trivial LP problem which has no rows */
      int j;
      double dir;
      xassert(lp->m == 0);
      switch (lp->dir)
      {  case GLP_MIN: dir = +1.0; break;
         case GLP_MAX: dir = -1.0; break;
         default:      xassert(lp != lp);
      }
      lp->pbs_stat = lp->dbs_stat = GLP_FEAS;
      lp->obj_val = lp->c0;
      lp->some = 0;
      for (j = 1; j <= lp->n; j++)
      {  GLPCOL *col = lp->col[j];
         col->dual = col->coef;
         switch (col->type)
         {  case GLP_FR:
               if (dir * col->dual < -1e-7)
                  lp->dbs_stat = GLP_NOFEAS;
               if (dir * col->dual > +1e-7)
                  lp->dbs_stat = GLP_NOFEAS;
               col->stat = GLP_NF;
               col->prim = 0.0;
               break;
            case GLP_LO:
               if (dir * col->dual < -1e-7)
                  lp->dbs_stat = GLP_NOFEAS;
lo:            col->stat = GLP_NL;
               col->prim = col->lb;
               break;
            case GLP_UP:
               if (dir * col->dual > +1e-7)
                  lp->dbs_stat = GLP_NOFEAS;
up:            col->stat = GLP_NU;
               col->prim = col->ub;
               break;
            case GLP_DB:
               if (dir * col->dual < 0.0) goto up;
               if (dir * col->dual > 0.0) goto lo;
               if (fabs(col->lb) <= fabs(col->ub))
                  goto lo;
               else
                  goto up;
            case GLP_FX:
               col->stat = GLP_NS;
               col->prim = col->lb;
               break;
            default:
               xassert(lp != lp);
         }
         lp->obj_val += col->coef * col->prim;
         if (lp->dbs_stat == GLP_NOFEAS && lp->some == 0)
            lp->some = j;
      }
      if (parm->msg_lev >= GLP_MSG_ON && parm->out_dly == 0)
      {  xprintf("~%6d:   objval = %17.9e   infeas = %17.9e\n",
            lp->it_cnt, lp->obj_val, 0.0);
         if (parm->msg_lev >= GLP_MSG_ALL)
         {  if (lp->dbs_stat == GLP_FEAS)
               xprintf("OPTIMAL SOLUTION FOUND\n");
            else
               xprintf("PROBLEM HAS UNBOUNDED SOLUTION\n");
         }
      }
      return;
}

static void trivial2(glp_prob *lp, const glp_smcp *parm)
{     /* solve trivial LP problem which has no columns */
      int i;
      xassert(lp->n == 0);
      lp->pbs_stat = lp->dbs_stat = GLP_FEAS;
      lp->obj_val = lp->c0;
      lp->some = 0;
      for (i = 1; i <= lp->m; i++)
      {  GLPROW *row = lp->row[i];
         row->stat = GLP_BS;
         row->prim = row->dual = 0.0;
         switch (row->type)
         {  case GLP_FR:
               break;
            case GLP_LO:
               if (row->lb > +1e-8)
                  lp->pbs_stat = GLP_NOFEAS;
               break;
            case GLP_UP:
               if (row->ub < -1e-8)
                  lp->pbs_stat = GLP_NOFEAS;
               break;
            case GLP_DB:
            case GLP_FX:
               if (row->lb > +1e-8)
                  lp->pbs_stat = GLP_NOFEAS;
               if (row->ub < -1e-8)
                  lp->pbs_stat = GLP_NOFEAS;
               break;
            default:
               xassert(lp != lp);
         }
      }
      if (parm->msg_lev >= GLP_MSG_ON && parm->out_dly == 0)
      {  xprintf("~%6d:   objval = %17.9e   infeas = %17.9e\n",
            lp->it_cnt, lp->obj_val);
         if (parm->msg_lev >= GLP_MSG_ALL)
         {  if (lp->pbs_stat == GLP_FEAS)
               xprintf("OPTIMAL SOLUTION FOUND\n");
            else
               xprintf("PROBLEM HAS NO FEASIBLE SOLUTION\n");
         }
      }
      return;
}

#if 1 /* 18/VIII-2008 */
static int simplex1(glp_prob *lp, const glp_smcp *parm)
{     /* base driver which does not use LP presolver */
      int ret;
      if (!glp_bf_exists(lp))
      {  ret = glp_factorize(lp);
         switch (ret)
         {  case 0:
               break;
            case GLP_EBADB:
               if (parm->msg_lev >= GLP_MSG_ERR)
                  xprintf("glp_simplex: initial basis is invalid\n");
               goto done;
            case GLP_ESING:
               if (parm->msg_lev >= GLP_MSG_ERR)
                  xprintf("glp_simplex: initial basis is singular\n");
               goto done;
            case GLP_ECOND:
               if (parm->msg_lev >= GLP_MSG_ERR)
                  xprintf("glp_simplex: initial basis is ill-conditione"
                     "d\n");
               goto done;
            default:
               xassert(ret != ret);
         }
      }
      switch (parm->meth)
      {  case GLP_PRIMAL:
            ret = spx_primal(lp, parm);
            break;
         case GLP_DUALP:
            {  glp_smcp parm1 = *parm;
#if 0
               parm1.msg_lev = GLP_MSG_DBG;
               parm1.out_frq = 1;
               parm1.out_dly = 0;
#endif
               ret = spx_dual(lp, &parm1);
#if 1
               if (ret == GLP_EFAIL && lp->valid)
                  ret = spx_primal(lp, parm);
#endif
            }
            break;
         case GLP_DUAL:
            ret = spx_dual(lp, parm);
            break;
         default:
            xassert(parm != parm);
      }
done: return ret;
}
#endif

static int simplex2(glp_prob *orig, const glp_smcp *parm)
{     /* extended driver which uses LP presolver */
#ifdef GLP_USE_NPP
      NPP *npp;
#else
      LPP *lpp;
#endif
      glp_prob *prob;
      glp_bfcp bfcp;
      int orig_m, orig_n, orig_nnz, ret;
      orig_m = glp_get_num_rows(orig);
      orig_n = glp_get_num_cols(orig);
      orig_nnz = glp_get_num_nz(orig);
      if (parm->msg_lev >= GLP_MSG_ALL)
      {  xprintf(
            "Original LP has %d row%s, %d column%s, %d non-zero%s\n",
            orig_m, orig_m == 1 ? "" : "s",
            orig_n, orig_n == 1 ? "" : "s",
            orig_nnz, orig_nnz == 1 ? "" : "s");
      }
      /* the problem must have at least one row and one column */
      xassert(orig_m > 0 && orig_n > 0);
      /* create LP presolver workspace */
#ifdef GLP_USE_NPP
      npp = npp_create_wksp();
#else
      lpp = lpp_create_wksp();
#endif
      /* load the original problem into LP presolver workspace */
#ifdef GLP_USE_NPP
      npp_load_prob(npp, orig, GLP_OFF, GLP_SOL, GLP_OFF);
#else
      lpp_load_orig(lpp, orig);
#endif
      /* perform LP presolve analysis */
#ifdef GLP_USE_NPP
      ret = npp_preprocess(npp);
#else
      ret = lpp_presolve(lpp);
#endif
      switch (ret)
      {  case 0:
            /* presolving has been successfully completed */
            break;
#ifdef GLP_USE_NPP
         case GLP_ENOPFS:
#else
         case 1:
#endif
            /* the original problem is primal infeasible */
            if (parm->msg_lev >= GLP_MSG_ALL)
               xprintf("PROBLEM HAS NO PRIMAL FEASIBLE SOLUTION\n");
#ifdef GLP_USE_NPP
            npp_delete_wksp(npp);
#else
            lpp_delete_wksp(lpp);
#endif
            return GLP_ENOPFS;
#ifdef GLP_USE_NPP
         case GLP_ENODFS:
#else
         case 2:
#endif
            /* the original problem is dual infeasible */
            if (parm->msg_lev >= GLP_MSG_ALL)
               xprintf("PROBLEM HAS NO DUAL FEASIBLE SOLUTION\n");
#ifdef GLP_USE_NPP
            npp_delete_wksp(npp);
#else
            lpp_delete_wksp(lpp);
#endif
            return GLP_ENODFS;
         default:
            xassert(ret != ret);
      }
      /* if the resultant problem is empty, it has an empty solution,
         which is optimal */
#ifdef GLP_USE_NPP
      if (npp->r_head == NULL || npp->c_head == NULL)
      {  xassert(npp->r_head == NULL);
         xassert(npp->c_head == NULL);
         if (parm->msg_lev >= GLP_MSG_ALL)
         {  xprintf("Objective value = %.10g\n",
               npp->orig_dir == GLP_MIN ? + npp->c0 : - npp->c0);
            xprintf("OPTIMAL SOLUTION FOUND BY LP PRESOLVER\n");
         }
         prob = glp_create_prob();
         npp_build_prob(npp, prob);
         xassert(prob->m == 0 && prob->n == 0);
         prob->pbs_stat = prob->dbs_stat = GLP_FEAS;
         prob->obj_val = prob->c0;
         goto post;
      }
#else
      if (lpp->row_ptr == NULL || lpp->col_ptr == NULL)
      {  xassert(lpp->row_ptr == NULL);
         xassert(lpp->col_ptr == NULL);
         if (parm->msg_lev >= GLP_MSG_ALL)
         {  xprintf("Objective value = %.10g\n",
               lpp->orig_dir == LPX_MIN ? + lpp->c0 : - lpp->c0);
            xprintf("OPTIMAL SOLUTION FOUND BY LP PRESOLVER\n");
         }
         /* allocate recovered solution segment */
         lpp_alloc_sol(lpp);
         goto post;
      }
#endif
      /* build resultant LP problem object */
#ifdef GLP_USE_NPP
      prob = glp_create_prob();
      npp_build_prob(npp, prob);
#else
      prob = lpp_build_prob(lpp);
#endif
      if (parm->msg_lev >= GLP_MSG_ALL)
      {  int m = glp_get_num_rows(prob);
         int n = glp_get_num_cols(prob);
         int nnz = glp_get_num_nz(prob);
         xprintf(
            "Presolved LP has %d row%s, %d column%s, %d non-zero%s\n",
            m, m == 1 ? "" : "s",
            n, n == 1 ? "" : "s",
            nnz, nnz == 1 ? "" : "s");
      }
      /* inherit basis factorization control parameters */
      glp_get_bfcp(orig, &bfcp);
      glp_set_bfcp(prob, &bfcp);
      /* scale the resultant problem */
      {  LIBENV *env = lib_link_env();
         int term_out = env->term_out;
         if (!term_out || parm->msg_lev < GLP_MSG_ALL)
            env->term_out = GLP_OFF;
         else
            env->term_out = GLP_ON;
         glp_scale_prob(prob, GLP_SF_AUTO);
         env->term_out = term_out;
      }
      /* build advanced initial basis */
      {  LIBENV *env = lib_link_env();
         int term_out = env->term_out;
         if (!term_out || parm->msg_lev < GLP_MSG_ALL)
            env->term_out = GLP_OFF;
         else
            env->term_out = GLP_ON;
         lpx_adv_basis(prob);
         env->term_out = term_out;
      }
      /* try to solve the resultant problem */
      prob->it_cnt = orig->it_cnt;
      ret = simplex1(prob, parm);
      orig->it_cnt = prob->it_cnt;
      /* check if the optimal solution has been found */
      if (glp_get_status(prob) != GLP_OPT)
      {  if (parm->msg_lev >= GLP_MSG_ERR)
            xprintf("glp_simplex: unable to recover undefined or non-op"
               "timal solution\n");
         if (ret == 0)
         {  if (glp_get_prim_stat(prob) == GLP_NOFEAS)
               ret = GLP_ENOPFS;
            else if (glp_get_dual_stat(prob) == GLP_NOFEAS)
               ret = GLP_ENODFS;
         }
         glp_delete_prob(prob);
#ifdef GLP_USE_NPP
         npp_delete_wksp(npp);
#else
         lpp_delete_wksp(lpp);
#endif
         return ret;
      }
#ifdef GLP_USE_NPP
post: npp_postprocess(npp, prob);
#else
      /* allocate recovered solution segment */
      lpp_alloc_sol(lpp);
      /* load basic solution of the resultant problem into LP presolver
         workspace */
      lpp_load_sol(lpp, prob);
#endif
      /* the resultant problem object is no longer needed */
      glp_delete_prob(prob);
#ifdef GLP_USE_NPP
      npp_unload_sol(npp, orig);
      npp_delete_wksp(npp);
#else
post: /* perform LP postsolve processing */
      lpp_postsolve(lpp);
      /* unload recovered basic solution and store it into the original
         problem object */
      lpp_unload_sol(lpp, orig);
      /* delete LP presolver workspace */
      lpp_delete_wksp(lpp);
#endif
      /* the original problem has been successfully solved */
      return 0;
}

int glp_simplex(glp_prob *lp, const glp_smcp *parm)
{     glp_smcp _parm;
      int i, j, ret;
      if (parm == NULL)
         parm = &_parm, glp_init_smcp((glp_smcp *)parm);
      /* check control parameters */
      if (!(parm->msg_lev == GLP_MSG_OFF ||
            parm->msg_lev == GLP_MSG_ERR ||
            parm->msg_lev == GLP_MSG_ON  ||
            parm->msg_lev == GLP_MSG_ALL))
         xerror("glp_simplex: msg_lev = %d; invalid parameter\n",
            parm->msg_lev);
      if (!(parm->meth == GLP_PRIMAL ||
            parm->meth == GLP_DUALP  ||
            parm->meth == GLP_DUAL))
         xerror("glp_simplex: meth = %d; invalid parameter\n",
            parm->meth);
      if (!(parm->pricing == GLP_PT_STD ||
            parm->pricing == GLP_PT_PSE))
         xerror("glp_simplex: pricing = %d; invalid parameter\n",
            parm->pricing);
      if (!(parm->r_test == GLP_RT_STD ||
            parm->r_test == GLP_RT_HAR))
         xerror("glp_simplex: r_test = %d; invalid parameter\n",
            parm->r_test);
      if (!(0.0 < parm->tol_bnd && parm->tol_bnd < 1.0))
         xerror("glp_simplex: tol_bnd = %g; invalid parameter\n",
            parm->tol_bnd);
      if (!(0.0 < parm->tol_dj && parm->tol_dj < 1.0))
         xerror("glp_simplex: tol_dj = %g; invalid parameter\n",
            parm->tol_dj);
      if (!(0.0 < parm->tol_piv && parm->tol_piv < 1.0))
         xerror("glp_simplex: tol_piv = %g; invalid parameter\n",
            parm->tol_piv);
      if (parm->it_lim < 0)
         xerror("glp_simplex: it_lim = %d; invalid parameter\n",
            parm->it_lim);
      if (parm->tm_lim < 0)
         xerror("glp_simplex: tm_lim = %d; invalid parameter\n",
            parm->tm_lim);
      if (parm->out_frq < 1)
         xerror("glp_simplex: out_frq = %d; invalid parameter\n",
            parm->out_frq);
      if (parm->out_dly < 0)
         xerror("glp_simplex: out_dly = %d; invalid parameter\n",
            parm->out_dly);
      if (!(parm->presolve == GLP_ON || parm->presolve == GLP_OFF))
         xerror("glp_simplex: presolve = %d; invalid parameter\n",
            parm->presolve);
      /* basic solution is currently undefined */
      lp->pbs_stat = lp->dbs_stat = GLP_UNDEF;
      lp->obj_val = 0.0;
      lp->some = 0;
      /* check bounds of double-bounded variables */
      for (i = 1; i <= lp->m; i++)
      {  GLPROW *row = lp->row[i];
         if (row->type == GLP_DB && row->lb >= row->ub)
         {  if (parm->msg_lev >= GLP_MSG_ERR)
               xprintf("glp_simplex: row %d: lb = %g, ub = %g; incorrec"
                  "t bounds\n", i, row->lb, row->ub);
            ret = GLP_EBOUND;
            goto done;
         }
      }
      for (j = 1; j <= lp->n; j++)
      {  GLPCOL *col = lp->col[j];
         if (col->type == GLP_DB && col->lb >= col->ub)
         {  if (parm->msg_lev >= GLP_MSG_ERR)
               xprintf("glp_simplex: column %d: lb = %g, ub = %g; incor"
                  "rect bounds\n", j, col->lb, col->ub);
            ret = GLP_EBOUND;
            goto done;
         }
      }
      /* solve LP problem */
      if (lp->m == 0)
         trivial1(lp, parm), ret = 0;
      else if (lp->n == 0)
         trivial2(lp, parm), ret = 0;
      else if (!parm->presolve)
         ret = simplex1(lp, parm);
      else
         ret = simplex2(lp, parm);
done: /* return to the application program */
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_init_smcp - initialize simplex method control parameters
*
*  SYNOPSIS
*
*  void glp_init_smcp(glp_smcp *parm);
*
*  DESCRIPTION
*
*  The routine glp_init_smcp initializes control parameters, which are
*  used by the simplex solver, with default values.
*
*  Default values of the control parameters are stored in a glp_smcp
*  structure, which the parameter parm points to. */

void glp_init_smcp(glp_smcp *parm)
{     parm->msg_lev = GLP_MSG_ALL;
      parm->meth = GLP_PRIMAL;
      parm->pricing = GLP_PT_PSE;
      parm->r_test = GLP_RT_HAR;
      parm->tol_bnd = 1e-7;
      parm->tol_dj = 1e-7;
      parm->tol_piv = 1e-10;
      parm->obj_ll = -DBL_MAX;
      parm->obj_ul = +DBL_MAX;
      parm->it_lim = INT_MAX;
      parm->tm_lim = INT_MAX;
      parm->out_frq = 200;
      parm->out_dly = 0;
      parm->presolve = GLP_OFF;
      return;
}

/***********************************************************************
*  NAME
*
*  glp_get_status - retrieve generic status of basic solution
*
*  SYNOPSIS
*
*  int glp_get_status(glp_prob *lp);
*
*  RETURNS
*
*  The routine glp_get_status reports the generic status of the basic
*  solution for the specified problem object as follows:
*
*  GLP_OPT    - solution is optimal;
*  GLP_FEAS   - solution is feasible;
*  GLP_INFEAS - solution is infeasible;
*  GLP_NOFEAS - problem has no feasible solution;
*  GLP_UNBND  - problem has unbounded solution;
*  GLP_UNDEF  - solution is undefined. */

int glp_get_status(glp_prob *lp)
{     int status;
      status = glp_get_prim_stat(lp);
      switch (status)
      {  case GLP_FEAS:
            switch (glp_get_dual_stat(lp))
            {  case GLP_FEAS:
                  status = GLP_OPT;
                  break;
               case GLP_NOFEAS:
                  status = GLP_UNBND;
                  break;
               case GLP_UNDEF:
               case GLP_INFEAS:
                  status = status;
                  break;
               default:
                  xassert(lp != lp);
            }
            break;
         case GLP_UNDEF:
         case GLP_INFEAS:
         case GLP_NOFEAS:
            status = status;
            break;
         default:
            xassert(lp != lp);
      }
      return status;
}

/***********************************************************************
*  NAME
*
*  glp_get_prim_stat - retrieve status of primal basic solution
*
*  SYNOPSIS
*
*  int glp_get_prim_stat(glp_prob *lp);
*
*  RETURNS
*
*  The routine glp_get_prim_stat reports the status of the primal basic
*  solution for the specified problem object as follows:
*
*  GLP_UNDEF  - primal solution is undefined;
*  GLP_FEAS   - primal solution is feasible;
*  GLP_INFEAS - primal solution is infeasible;
*  GLP_NOFEAS - no primal feasible solution exists. */

int glp_get_prim_stat(glp_prob *lp)
{     int pbs_stat = lp->pbs_stat;
      return pbs_stat;
}

/***********************************************************************
*  NAME
*
*  glp_get_dual_stat - retrieve status of dual basic solution
*
*  SYNOPSIS
*
*  int glp_get_dual_stat(glp_prob *lp);
*
*  RETURNS
*
*  The routine glp_get_dual_stat reports the status of the dual basic
*  solution for the specified problem object as follows:
*
*  GLP_UNDEF  - dual solution is undefined;
*  GLP_FEAS   - dual solution is feasible;
*  GLP_INFEAS - dual solution is infeasible;
*  GLP_NOFEAS - no dual feasible solution exists. */

int glp_get_dual_stat(glp_prob *lp)
{     int dbs_stat = lp->dbs_stat;
      return dbs_stat;
}

/***********************************************************************
*  NAME
*
*  glp_get_obj_val - retrieve objective value (basic solution)
*
*  SYNOPSIS
*
*  double glp_get_obj_val(glp_prob *lp);
*
*  RETURNS
*
*  The routine glp_get_obj_val returns value of the objective function
*  for basic solution. */

double glp_get_obj_val(glp_prob *lp)
{     struct LPXCPS *cps = lp->cps;
      double z;
      z = lp->obj_val;
      if (cps->round && fabs(z) < 1e-9) z = 0.0;
      return z;
}

/***********************************************************************
*  NAME
*
*  glp_get_row_stat - retrieve row status
*
*  SYNOPSIS
*
*  int glp_get_row_stat(glp_prob *lp, int i);
*
*  RETURNS
*
*  The routine glp_get_row_stat returns current status assigned to the
*  auxiliary variable associated with i-th row as follows:
*
*  GLP_BS - basic variable;
*  GLP_NL - non-basic variable on its lower bound;
*  GLP_NU - non-basic variable on its upper bound;
*  GLP_NF - non-basic free (unbounded) variable;
*  GLP_NS - non-basic fixed variable. */

int glp_get_row_stat(glp_prob *lp, int i)
{     if (!(1 <= i && i <= lp->m))
         xerror("glp_get_row_stat: i = %d; row number out of range\n",
            i);
      return lp->row[i]->stat;
}

/***********************************************************************
*  NAME
*
*  glp_get_row_prim - retrieve row primal value (basic solution)
*
*  SYNOPSIS
*
*  double glp_get_row_prim(glp_prob *lp, int i);
*
*  RETURNS
*
*  The routine glp_get_row_prim returns primal value of the auxiliary
*  variable associated with i-th row. */

double glp_get_row_prim(glp_prob *lp, int i)
{     struct LPXCPS *cps = lp->cps;
      double prim;
      if (!(1 <= i && i <= lp->m))
         xerror("glp_get_row_prim: i = %d; row number out of range\n",
            i);
      prim = lp->row[i]->prim;
      if (cps->round && fabs(prim) < 1e-9) prim = 0.0;
      return prim;
}

/***********************************************************************
*  NAME
*
*  glp_get_row_dual - retrieve row dual value (basic solution)
*
*  SYNOPSIS
*
*  double glp_get_row_dual(glp_prob *lp, int i);
*
*  RETURNS
*
*  The routine glp_get_row_dual returns dual value (i.e. reduced cost)
*  of the auxiliary variable associated with i-th row. */

double glp_get_row_dual(glp_prob *lp, int i)
{     struct LPXCPS *cps = lp->cps;
      double dual;
      if (!(1 <= i && i <= lp->m))
         xerror("glp_get_row_dual: i = %d; row number out of range\n",
            i);
      dual = lp->row[i]->dual;
      if (cps->round && fabs(dual) < 1e-9) dual = 0.0;
      return dual;
}

/***********************************************************************
*  NAME
*
*  glp_get_col_stat - retrieve column status
*
*  SYNOPSIS
*
*  int glp_get_col_stat(glp_prob *lp, int j);
*
*  RETURNS
*
*  The routine glp_get_col_stat returns current status assigned to the
*  structural variable associated with j-th column as follows:
*
*  GLP_BS - basic variable;
*  GLP_NL - non-basic variable on its lower bound;
*  GLP_NU - non-basic variable on its upper bound;
*  GLP_NF - non-basic free (unbounded) variable;
*  GLP_NS - non-basic fixed variable. */

int glp_get_col_stat(glp_prob *lp, int j)
{     if (!(1 <= j && j <= lp->n))
         xerror("glp_get_col_stat: j = %d; column number out of range\n"
            , j);
      return lp->col[j]->stat;
}

/***********************************************************************
*  NAME
*
*  glp_get_col_prim - retrieve column primal value (basic solution)
*
*  SYNOPSIS
*
*  double glp_get_col_prim(glp_prob *lp, int j);
*
*  RETURNS
*
*  The routine glp_get_col_prim returns primal value of the structural
*  variable associated with j-th column. */

double glp_get_col_prim(glp_prob *lp, int j)
{     struct LPXCPS *cps = lp->cps;
      double prim;
      if (!(1 <= j && j <= lp->n))
         xerror("glp_get_col_prim: j = %d; column number out of range\n"
            , j);
      prim = lp->col[j]->prim;
      if (cps->round && fabs(prim) < 1e-9) prim = 0.0;
      return prim;
}

/***********************************************************************
*  NAME
*
*  glp_get_col_dual - retrieve column dual value (basic solution)
*
*  SYNOPSIS
*
*  double glp_get_col_dual(glp_prob *lp, int j);
*
*  RETURNS
*
*  The routine glp_get_col_dual returns dual value (i.e. reduced cost)
*  of the structural variable associated with j-th column. */

double glp_get_col_dual(glp_prob *lp, int j)
{     struct LPXCPS *cps = lp->cps;
      double dual;
      if (!(1 <= j && j <= lp->n))
         xerror("glp_get_col_dual: j = %d; column number out of range\n"
            , j);
      dual = lp->col[j]->dual;
      if (cps->round && fabs(dual) < 1e-9) dual = 0.0;
      return dual;
}

/***********************************************************************
*  NAME
*
*  glp_get_unbnd_ray - determine variable causing unboundedness
*
*  SYNOPSIS
*
*  int glp_get_unbnd_ray(glp_prob *lp);
*
*  RETURNS
*
*  The routine glp_get_unbnd_ray returns the number k of a variable,
*  which causes primal or dual unboundedness. If 1 <= k <= m, it is
*  k-th auxiliary variable, and if m+1 <= k <= m+n, it is (k-m)-th
*  structural variable, where m is the number of rows, n is the number
*  of columns in the problem object. If such variable is not defined,
*  the routine returns 0.
*
*  COMMENTS
*
*  If it is not exactly known which version of the simplex solver
*  detected unboundedness, i.e. whether the unboundedness is primal or
*  dual, it is sufficient to check the status of the variable reported
*  with the routine glp_get_row_stat or glp_get_col_stat. If the
*  variable is non-basic, the unboundedness is primal, otherwise, if
*  the variable is basic, the unboundedness is dual (the latter case
*  means that the problem has no primal feasible dolution). */

int glp_get_unbnd_ray(glp_prob *lp)
{     int k;
      k = lp->some;
      xassert(k >= 0);
      if (k > lp->m + lp->n) k = 0;
      return k;
}

/* eof */
