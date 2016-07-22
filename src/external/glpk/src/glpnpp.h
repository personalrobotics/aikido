/* glpnpp.h (LP/MIP preprocessor) */

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

#ifndef GLPNPP_H
#define GLPNPP_H

#include "glpapi.h"

typedef struct NPP NPP;
typedef struct NPPROW NPPROW;
typedef struct NPPCOL NPPCOL;
typedef struct NPPAIJ NPPAIJ;
typedef struct NPPTSE NPPTSE;
typedef struct NPPLFE NPPLFE;
typedef struct NPPLFX NPPLFX;

struct NPP
{     /* LP/MIP preprocessor workspace */
      /*--------------------------------------------------------------*/
      /* original problem segment */
      int orig_dir;
      /* optimization direction flag:
         GLP_MIN - minimization
         GLP_MAX - maximization */
      int orig_m;
      /* number of rows */
      int orig_n;
      /* number of columns */
      int orig_nnz;
      /* number of non-zero constraint coefficients */
      /*--------------------------------------------------------------*/
      /* transformed problem segment (always minimization) */
      DMP *pool;
      /* memory pool to store problem components */
      char *name;
      /* problem name (1 to 255 chars); NULL means no name is assigned
         to the problem */
      char *obj;
      /* objective function name (1 to 255 chars); NULL means no name
         is assigned to the objective function */
      double c0;
      /* constant term of the objective function */
      int nrows;
      /* number of rows introduced into the problem; this count
         increases by one every time a new row is added and never
         decreases; thus, actual number of rows may be less than nrows
         due to row deletions */
      int ncols;
      /* number of columns introduced into the problem; this count
         increases by one every time a new column is added and never
         decreases; thus, actual number of column may be less than
         ncols due to column deletions */
      NPPROW *r_head;
      /* pointer to the beginning of the row list */
      NPPROW *r_tail;
      /* pointer to the end of the row list */
      NPPCOL *c_head;
      /* pointer to the beginning of the column list */
      NPPCOL *c_tail;
      /* pointer to the end of the column list */
      /*--------------------------------------------------------------*/
      /* transformation history segment */
      DMP *stack;
      /* memory pool to store transformation entries */
      NPPTSE *top;
      /* pointer to most recent transformation entry */
      /*--------------------------------------------------------------*/
      /* resultant (preprocessed) problem segment */
      int m;
      /* number of rows */
      int n;
      /* number of columns */
      int nnz;
      /* number of non-zero constraint coefficients */
      int *row_ref; /* int row_ref[1+m]; */
      /* row_ref[i], 1 <= i <= m, is the reference number assigned to
         a row, which is i-th row of the resultant problem */
      int *col_ref; /* int col_ref[1+n]; */
      /* col_ref[j], 1 <= j <= n, is the reference number assigned to
         a column, which is j-th column of the resultant problem */
      /*--------------------------------------------------------------*/
      /* recovered solution segment */
      int sol;
      /* solution indicator:
         GLP_SOL - basic solution
         GLP_IPT - interior-point solution
         GLP_MIP - mixed integer solution */
      int scaling;
      /* scaling option:
         GLP_OFF - scaling is disabled
         GLP_ON  - scaling is enabled */
      int p_stat;
      /* status of primal basic solution:
         GLP_UNDEF  - primal solution is undefined
         GLP_FEAS   - primal solution is feasible
         GLP_INFEAS - primal solution is infeasible
         GLP_NOFEAS - no primal feasible solution exists */
      int d_stat;
      /* status of dual basic solution:
         GLP_UNDEF  - dual solution is undefined
         GLP_FEAS   - dual solution is feasible
         GLP_INFEAS - dual solution is infeasible
         GLP_NOFEAS - no dual feasible solution exists */
      int t_stat;
      /* status of interior-point solution:
         GLP_UNDEF  - interior solution is undefined
         GLP_OPT    - interior solution is optimal */
      int i_stat;
      /* status of mixed integer solution:
         GLP_UNDEF  - integer solution is undefined
         GLP_OPT    - integer solution is optimal
         GLP_FEAS   - integer solution is feasible
         GLP_NOFEAS - no integer solution exists */
      char *r_stat; /* char r_stat[1+nrows]; */
      /* r_stat[i], 1 <= i <= nrows, is the status of i-th row:
         GLP_BS - inactive constraint
         GLP_NL - active constraint on lower bound
         GLP_NU - active constraint on upper bound
         GLP_NF - active free row
         GLP_NS - active equality constraint */
      double *r_prim; /* double r_prim[1+nrows]; */
      /* r_prim[i], 1 <= i <= nrows, is a primal value of i-th row */
      double *r_dual; /* double r_dual[1+nrows]; */
      /* r_dual[i], 1 <= i <= nrows, is a dual value of i-th row */
      char *c_stat; /* char c_stat[1+nrows]; */
      /* c_stat[j], 1 <= j <= nrows, is the status of j-th column:
         GLP_BS - basic variable
         GLP_NL - non-basic variable on lower bound
         GLP_NU - non-basic variable on upper bound
         GLP_NF - non-basic free variable
         GLP_NS - non-basic fixed variable */
      double *c_prim; /* double c_prim[1+ncols]; */
      /* c_prim[j], 1 <= j <= ncols, is a primal value of j-th column */
      double *c_dual; /* double c_dual[1+ncols]; */
      /* c_dual[j], 1 <= j <= ncols, is a dual value of j-th column */
};

struct NPPROW
{     /* row (constraint) */
      int i;
      /* reference number assigned to the row, 1 <= i <= nrows */
      char *name;
      /* row name (1 to 255 chars); NULL means no name is assigned to
         the row */
      double lb;
      /* lower bound; -DBL_MAX means the row has no lower bound */
      double ub;
      /* upper bound; +DBL_MAX means the row has no upper bound */
      NPPAIJ *ptr;
      /* pointer to the linked list of constraint coefficients */
      int temp;
      /* working field used by preprocessor routines */
      NPPROW *prev;
      /* pointer to previous row in the row list */
      NPPROW *next;
      /* pointer to next row in the row list */
};

struct NPPCOL
{     /* column (variable) */
      int j;
      /* reference number assigned to the column, 1 <= j <= ncols */
      char *name;
      /* column name (1 to 255 chars); NULL means no name is assigned
         to the column */
      int kind;
      /* column kind:
         GLP_CV - continuous variable
         GLP_IV - integer variable */
      double lb;
      /* lower bound; -DBL_MAX means the column has no lower bound */
      double ub;
      /* upper bound; +DBL_MAX means the column has no upper bound */
      double coef;
      /* objective coefficient */
      NPPAIJ *ptr;
      /* pointer to the linked list of constraint coefficients */
      int temp;
      /* working field used by preprocessor routines */
      NPPCOL *prev;
      /* pointer to previous column in the column list */
      NPPCOL *next;
      /* pointer to next column in the column list */
};

struct NPPAIJ
{     /* constraint coefficient */
      NPPROW *row;
      /* pointer to corresponding row */
      NPPCOL *col;
      /* pointer to corresponding column */
      double val;
      /* (non-zero) coefficient value */
      NPPAIJ *r_prev;
      /* pointer to previous coefficient in the same row */
      NPPAIJ *r_next;
      /* pointer to next coefficient in the same row */
      NPPAIJ *c_prev;
      /* pointer to previous coefficient in the same column */
      NPPAIJ *c_next;
      /* pointer to next coefficient in the same column */
};

struct NPPTSE
{     /* transformation stack entry */
      void (*func)(NPP *npp, void *info);
      /* pointer to routine performing back transformation */
      void *info;
      /* pointer to specific info (depends on the transformation) */
      NPPTSE *link;
      /* pointer to another entry created *before* this entry */
};

struct NPPLFE
{     /* linear form element */
      int ref;
      /* row/column reference number */
      double val;
      /* (non-zero) coefficient value */
      NPPLFE *next;
      /* pointer to another element */
};

struct NPPLFX
{     /* extended linear form element */
      int ref;
      /* row/column reference number */
      char flag;
      /* row/column flag */
      double val;
      /* (non-zero) coefficient value */
      NPPLFX *next;
      /* pointer to another element */
};

#define npp_create_wksp _glp_npp_create_wksp
NPP *npp_create_wksp(void);
/* create LP/MIP preprocessor workspace */

#define npp_insert_row _glp_npp_insert_row
void npp_insert_row(NPP *npp, NPPROW *row, int where);
/* insert row to the row list */

#define npp_remove_row _glp_npp_remove_row
void npp_remove_row(NPP *npp, NPPROW *row);
/* remove row from the row list */

#define npp_insert_col _glp_npp_insert_col
void npp_insert_col(NPP *npp, NPPCOL *col, int where);
/* insert column to the column list */

#define npp_remove_col _glp_npp_remove_col
void npp_remove_col(NPP *npp, NPPCOL *col);
/* remove column from the column list */

#define npp_add_row _glp_npp_add_row
NPPROW *npp_add_row(NPP *npp);
/* add new row to the transformed problem */

#define npp_add_col _glp_npp_add_col
NPPCOL *npp_add_col(NPP *npp);
/* add new column to the transformed problem */

#define npp_add_aij _glp_npp_add_aij
NPPAIJ *npp_add_aij(NPP *npp, NPPROW *row, NPPCOL *col, double val);
/* add new element to the constraint matrix */

#define npp_push_tse _glp_npp_push_tse
void *npp_push_tse(NPP *npp, void (*func)(NPP *npp, void *info),
      int size);
/* push new entry to the transformation stack */

#define npp_del_row _glp_npp_del_row
void npp_del_row(NPP *npp, NPPROW *row);
/* remove row from the transformed problem */

#define npp_del_col _glp_npp_del_col
void npp_del_col(NPP *npp, NPPCOL *col);
/* remove column from the transformed problem */

#define npp_load_prob _glp_npp_load_prob
void npp_load_prob(NPP *npp, glp_prob *orig, int names, int sol,
      int scaling);
/* load original problem into the preprocessor workspace */

#define npp_build_prob _glp_npp_build_prob
void npp_build_prob(NPP *npp, glp_prob *prob);
/* build resultant (preprocessed) problem */

#define npp_postprocess _glp_npp_postprocess
void npp_postprocess(NPP *npp, glp_prob *prob);
/* postprocess solution from the resultant problem */

#define npp_unload_sol _glp_npp_unload_sol
void npp_unload_sol(NPP *npp, glp_prob *orig);
/* store solution to the original problem */

#define npp_delete_wksp _glp_npp_delete_wksp
void npp_delete_wksp(NPP *npp);
/* delete LP/MIP preprocessor workspace */

#define npp_free_row _glp_npp_free_row
void npp_free_row(NPP *npp, NPPROW *row);
/* process free row */

#define npp_gteq_row _glp_npp_gteq_row
void npp_gteq_row(NPP *npp, NPPROW *row);
/* process row of 'greater than or equal to' type */

#define npp_lteq_row _glp_npp_lteq_row
void npp_lteq_row(NPP *npp, NPPROW *row);
/* process row of 'less than or equal to' type */

#define npp_free_col _glp_npp_free_col
void npp_free_col(NPP *npp, NPPCOL *col);
/* process free column */

#define npp_lbnd_col _glp_npp_lbnd_col
void npp_lbnd_col(NPP *npp, NPPCOL *col);
/* process column with lower bound */

#define npp_ubnd_col _glp_npp_ubnd_col
void npp_ubnd_col(NPP *npp, NPPCOL *col);
/* process column with upper bound */

#define npp_dbnd_col _glp_npp_dbnd_col
void npp_dbnd_col(NPP *npp, NPPCOL *col);
/* process double-bounded column */

#define npp_fixed_col _glp_npp_fixed_col
void npp_fixed_col(NPP *npp, NPPCOL *col);
/* process fixed column */

#define npp_empty_row _glp_npp_empty_row
int npp_empty_row(NPP *npp, NPPROW *row);
/* process empty row */

#define npp_empty_col _glp_npp_empty_col
int npp_empty_col(NPP *npp, NPPCOL *col);
/* process empty column */

#define npp_implied_fixed _glp_npp_implied_fixed
int npp_implied_fixed(NPP *npp, NPPCOL *col, double val);
/* process implied fixed value of column */

#define npp_row_sngtn1 _glp_npp_row_sngtn1
int npp_row_sngtn1(NPP *npp, NPPROW *row);
/* process row singleton (equality constraint) */

#define npp_implied_lower _glp_npp_implied_lower
int npp_implied_lower(NPP *npp, NPPCOL *col, double bnd);
/* process implied lower bound of column */

#define npp_implied_upper _glp_npp_implied_upper
int npp_implied_upper(NPP *npp, NPPCOL *col, double bnd);
/* process implied upper bound of column */

#define npp_row_sngtn2 _glp_npp_row_sngtn2
int npp_row_sngtn2(NPP *npp, NPPROW *row);
/* process row singleton (inequality constraint) */

#define npp_col_sngtn1 _glp_npp_col_sngtn1
void npp_col_sngtn1(NPP *npp, NPPCOL *col);
/* process column singleton (implied slack variable) */

#define npp_col_sngtn2 _glp_npp_col_sngtn2
int npp_col_sngtn2(NPP *npp, NPPCOL *col);
/* process column singleton (implied free variable) */

#define npp_forcing_row _glp_npp_forcing_row
int npp_forcing_row(NPP *npp, NPPROW *row, int at);
/* process forcing row */

#define npp_preprocess _glp_npp_preprocess
int npp_preprocess(NPP *npp);
/* preprocess LP/MIP instance */

#endif

/* eof */
