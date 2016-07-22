/* glpdmx.c (reading/writing data in DIMACS format) */

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

#define _GLPSTD_STDIO
#include "glpapi.h"

struct csa
{     /* common storage area */
      jmp_buf jump;
      /* label for go to in case of error */
      const char *fname;
      /* name of input text file */
      XFILE *fp;
      /* stream assigned to input text file */
      int count;
      /* line count */
      int c;
      /* current character */
      char field[31+1];
      /* data field */
      int empty;
      /* warning 'empty line ignored' was printed */
      int nonint;
      /* warning 'non-integer data detected' was printed */
};

static void error(struct csa *csa, const char *fmt, ...)
{     /* print error message and terminate processing */
      va_list arg;
      xprintf("%s:%d: error: ", csa->fname, csa->count);
      va_start(arg, fmt);
      xvprintf(fmt, arg);
      va_end(arg);
      xprintf("\n");
      longjmp(csa->jump, 1);
      /* no return */
}

static void warning(struct csa *csa, const char *fmt, ...)
{     /* print warning message and continue processing */
      va_list arg;
      xprintf("%s:%d: warning: ", csa->fname, csa->count);
      va_start(arg, fmt);
      xvprintf(fmt, arg);
      va_end(arg);
      xprintf("\n");
      return;
}

static void read_char(struct csa *csa)
{     /* read character from input text file */
      int c;
      if (csa->c == '\n') csa->count++;
      c = xfgetc(csa->fp);
      if (c < 0)
      {  if (xferror(csa->fp))
            error(csa, "read error - %s", xerrmsg());
         else if (csa->c == '\n')
            error(csa, "unexpected end of file");
         else
         {  warning(csa, "missing final end of line");
            c = '\n';
         }
      }
      else if (c == '\n')
         ;
      else if (isspace(c))
         c = ' ';
      else if (iscntrl(c))
         error(csa, "invalid control character 0x%02X", c);
      csa->c = c;
      return;
}

static void read_designator(struct csa *csa)
{     /* read one-character line designator */
      xassert(csa->c == '\n');
      read_char(csa);
      for (;;)
      {  /* skip preceding white-space characters */
         while (csa->c == ' ')
            read_char(csa);
         if (csa->c == '\n')
         {  /* ignore empty line */
            if (!csa->empty)
            {  warning(csa, "empty line ignored");
               csa->empty = 1;
            }
            read_char(csa);
         }
         else if (csa->c == 'c')
         {  /* skip comment line */
            while (csa->c != '\n')
               read_char(csa);
            read_char(csa);
         }
         else
         {  /* hmm..., looks like a line designator */
            csa->field[0] = (char)csa->c, csa->field[1] = '\0';
            /* check that it is followed by a white-space character */
            read_char(csa);
            if (!(csa->c == ' ' || csa->c == '\n'))
               error(csa, "line designator missing or invalid");
            break;
         }
      }
      return;
}

static void read_field(struct csa *csa)
{     /* read data field */
      int len = 0;
      /* skip preceding white-space characters */
      while (csa->c == ' ')
         read_char(csa);
      /* scan data field */
      if (csa->c == '\n')
         error(csa, "unexpected end of line");
      while (!(csa->c == ' ' || csa->c == '\n'))
      {  if (len == sizeof(csa->field)-1)
            error(csa, "data field `%.*s...' too long",
               sizeof(csa->field)-1, csa->field);
         csa->field[len++] = (char)csa->c;
         read_char(csa);
      }
      csa->field[len] = '\0';
      return;
}

static void end_of_line(struct csa *csa)
{     /* skip white-space characters until end of line */
      while (csa->c == ' ')
         read_char(csa);
      if (csa->c != '\n')
         error(csa, "too many data fields specified");
      return;
}

static void check_int(struct csa *csa, double num)
{     /* print a warning if non-integer data are detected */
      if (!csa->nonint && num != floor(num))
      {  warning(csa, "non-integer data detected");
         csa->nonint = 1;
      }
      return;
}

/***********************************************************************
*  NAME
*
*  glp_read_mincost - read min-cost flow problem data in DIMACS format
*
*  SYNOPSIS
*
*  int glp_read_mincost(glp_graph *G, int v_rhs, int a_low, int a_cap,
*     int a_cost, const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_read_mincost reads minimum cost flow problem data in
*  DIMACS format from a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_read_mincost(glp_graph *G, int v_rhs, int a_low, int a_cap,
      int a_cost, const char *fname)
{     struct csa _csa, *csa = &_csa;
      glp_vertex *v;
      glp_arc *a;
      int i, j, k, nv, na, ret = 0;
      double rhs, low, cap, cost;
      char *flag = NULL;
      if (v_rhs >= 0 && v_rhs > G->v_size - (int)sizeof(double))
         xerror("glp_read_mincost: v_rhs = %d; invalid offset\n",
            v_rhs);
      if (a_low >= 0 && a_low > G->a_size - (int)sizeof(double))
         xerror("glp_read_mincost: a_low = %d; invalid offset\n",
            a_low);
      if (a_cap >= 0 && a_cap > G->a_size - (int)sizeof(double))
         xerror("glp_read_mincost: a_cap = %d; invalid offset\n",
            a_cap);
      if (a_cost >= 0 && a_cost > G->a_size - (int)sizeof(double))
         xerror("glp_read_mincost: a_cost = %d; invalid offset\n",
            a_cost);
      glp_erase_graph(G, G->v_size, G->a_size);
      if (setjmp(csa->jump))
      {  ret = 1;
         goto done;
      }
      csa->fname = fname;
      csa->fp = NULL;
      csa->count = 0;
      csa->c = '\n';
      csa->field[0] = '\0';
      csa->empty = csa->nonint = 0;
      xprintf("Reading min-cost flow problem data from `%s'...\n",
         fname);
      csa->fp = xfopen(fname, "r");
      if (csa->fp == NULL)
      {  xprintf("Unable to open `%s' - %s\n", fname, xerrmsg());
         longjmp(csa->jump, 1);
      }
      /* read problem line */
      read_designator(csa);
      if (strcmp(csa->field, "p") != 0)
         error(csa, "problem line missing or invalid");
      read_field(csa);
      if (strcmp(csa->field, "min") != 0)
         error(csa, "wrong problem designator; `min' expected");
      read_field(csa);
      if (!(str2int(csa->field, &nv) == 0 && nv >= 0))
         error(csa, "number of nodes missing or invalid");
      read_field(csa);
      if (!(str2int(csa->field, &na) == 0 && na >= 0))
         error(csa, "number of arcs missing or invalid");
      xprintf("Flow network has %d node%s and %d arc%s\n",
         nv, nv == 1 ? "" : "s", na, na == 1 ? "" : "s");
      if (nv > 0) glp_add_vertices(G, nv);
      end_of_line(csa);
      /* read node descriptor lines */
      flag = xcalloc(1+nv, sizeof(char));
      memset(&flag[1], 0, nv * sizeof(char));
      if (v_rhs >= 0)
      {  rhs = 0.0;
         for (i = 1; i <= nv; i++)
         {  v = G->v[i];
            memcpy((char *)v->data + v_rhs, &rhs, sizeof(double));
         }
      }
      for (;;)
      {  read_designator(csa);
         if (strcmp(csa->field, "n") != 0) break;
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "node number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "node number %d out of range", i);
         if (flag[i])
            error(csa, "duplicate descriptor of node %d", i);
         read_field(csa);
         if (str2num(csa->field, &rhs) != 0)
            error(csa, "node supply/demand missing or invalid");
         check_int(csa, rhs);
         if (v_rhs >= 0)
         {  v = G->v[i];
            memcpy((char *)v->data + v_rhs, &rhs, sizeof(double));
         }
         flag[i] = 1;
         end_of_line(csa);
      }
      xfree(flag), flag = NULL;
      /* read arc descriptor lines */
      for (k = 1; k <= na; k++)
      {  if (k > 1) read_designator(csa);
         if (strcmp(csa->field, "a") != 0)
            error(csa, "wrong line designator; `a' expected");
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "starting node number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "starting node number %d out of range", i);
         read_field(csa);
         if (str2int(csa->field, &j) != 0)
            error(csa, "ending node number missing or invalid");
         if (!(1 <= j && j <= nv))
            error(csa, "ending node number %d out of range", j);
         read_field(csa);
         if (!(str2num(csa->field, &low) == 0 && low >= 0.0))
            error(csa, "lower bound of arc flow missing or invalid");
         check_int(csa, low);
         read_field(csa);
         if (!(str2num(csa->field, &cap) == 0 && cap >= low))
            error(csa, "upper bound of arc flow missing or invalid");
         check_int(csa, cap);
         read_field(csa);
         if (str2num(csa->field, &cost) != 0)
            error(csa, "per-unit cost of arc flow missing or invalid");
         check_int(csa, cost);
         a = glp_add_arc(G, i, j);
         if (a_low >= 0)
            memcpy((char *)a->data + a_low, &low, sizeof(double));
         if (a_cap >= 0)
            memcpy((char *)a->data + a_cap, &cap, sizeof(double));
         if (a_cost >= 0)
            memcpy((char *)a->data + a_cost, &cost, sizeof(double));
         end_of_line(csa);
      }
      xprintf("%d lines were read\n", csa->count);
done: if (ret) glp_erase_graph(G, G->v_size, G->a_size);
      if (csa->fp != NULL) xfclose(csa->fp);
      if (flag != NULL) xfree(flag);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_write_mincost - write min-cost flow problem data in DIMACS format
*
*  SYNOPSIS
*
*  int glp_write_mincost(glp_graph *G, int v_rhs, int a_low, int a_cap,
*     int a_cost, const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_write_mincost writes minimum cost flow problem data
*  in DIMACS format to a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_write_mincost(glp_graph *G, int v_rhs, int a_low, int a_cap,
      int a_cost, const char *fname)
{     XFILE *fp;
      glp_vertex *v;
      glp_arc *a;
      int i, count = 0, ret;
      double rhs, low, cap, cost;
      if (v_rhs >= 0 && v_rhs > G->v_size - (int)sizeof(double))
         xerror("glp_write_mincost: v_rhs = %d; invalid offset\n",
            v_rhs);
      if (a_low >= 0 && a_low > G->a_size - (int)sizeof(double))
         xerror("glp_write_mincost: a_low = %d; invalid offset\n",
            a_low);
      if (a_cap >= 0 && a_cap > G->a_size - (int)sizeof(double))
         xerror("glp_write_mincost: a_cap = %d; invalid offset\n",
            a_cap);
      if (a_cost >= 0 && a_cost > G->a_size - (int)sizeof(double))
         xerror("glp_write_mincost: a_cost = %d; invalid offset\n",
            a_cost);
      xprintf("Writing min-cost flow problem data to `%s'...\n",
         fname);
      fp = xfopen(fname, "w");
      if (fp == NULL)
      {  xprintf("Unable to create `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xfprintf(fp, "c %s\n",
         G->name == NULL ? "unknown" : G->name), count++;
      xfprintf(fp, "p min %d %d\n", G->nv, G->na), count++;
      if (v_rhs >= 0)
      {  for (i = 1; i <= G->nv; i++)
         {  v = G->v[i];
            memcpy(&rhs, (char *)v->data + v_rhs, sizeof(double));
            if (rhs != 0.0)
               xfprintf(fp, "n %d %.*g\n", i, DBL_DIG, rhs), count++;
         }
      }
      for (i = 1; i <= G->nv; i++)
      {  v = G->v[i];
         for (a = v->out; a != NULL; a = a->t_next)
         {  if (a_low >= 0)
               memcpy(&low, (char *)a->data + a_low, sizeof(double));
            else
               low = 0.0;
            if (a_cap >= 0)
               memcpy(&cap, (char *)a->data + a_cap, sizeof(double));
            else
               cap = 1.0;
            if (a_cost >= 0)
               memcpy(&cost, (char *)a->data + a_cost, sizeof(double));
            else
               cost = 0.0;
            xfprintf(fp, "a %d %d %.*g %.*g %.*g\n",
               a->tail->i, a->head->i, DBL_DIG, low, DBL_DIG, cap,
               DBL_DIG, cost), count++;
         }
      }
      xfprintf(fp, "c eof\n"), count++;
      xfflush(fp);
      if (xferror(fp))
      {  xprintf("Write error on `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xprintf("%d lines were written\n", count);
      ret = 0;
done: if (fp != NULL) xfclose(fp);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_read_maxflow - read maximum flow problem data in DIMACS format
*
*  SYNOPSIS
*
*  int glp_read_maxflow(glp_graph *G, int *s, int *t, int a_cap,
*     const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_read_maxflow reads maximum flow problem data in
*  DIMACS format from a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_read_maxflow(glp_graph *G, int *_s, int *_t, int a_cap,
      const char *fname)
{     struct csa _csa, *csa = &_csa;
      glp_arc *a;
      int i, j, k, s, t, nv, na, ret = 0;
      double cap;
      if (a_cap >= 0 && a_cap > G->a_size - (int)sizeof(double))
         xerror("glp_read_maxflow: a_cap = %d; invalid offset\n",
            a_cap);
      glp_erase_graph(G, G->v_size, G->a_size);
      if (setjmp(csa->jump))
      {  ret = 1;
         goto done;
      }
      csa->fname = fname;
      csa->fp = NULL;
      csa->count = 0;
      csa->c = '\n';
      csa->field[0] = '\0';
      csa->empty = csa->nonint = 0;
      xprintf("Reading maximum flow problem data from `%s'...\n",
         fname);
      csa->fp = xfopen(fname, "r");
      if (csa->fp == NULL)
      {  xprintf("Unable to open `%s' - %s\n", fname, xerrmsg());
         longjmp(csa->jump, 1);
      }
      /* read problem line */
      read_designator(csa);
      if (strcmp(csa->field, "p") != 0)
         error(csa, "problem line missing or invalid");
      read_field(csa);
      if (strcmp(csa->field, "max") != 0)
         error(csa, "wrong problem designator; `max' expected");
      read_field(csa);
      if (!(str2int(csa->field, &nv) == 0 && nv >= 2))
         error(csa, "number of nodes missing or invalid");
      read_field(csa);
      if (!(str2int(csa->field, &na) == 0 && na >= 0))
         error(csa, "number of arcs missing or invalid");
      xprintf("Flow network has %d node%s and %d arc%s\n",
         nv, nv == 1 ? "" : "s", na, na == 1 ? "" : "s");
      if (nv > 0) glp_add_vertices(G, nv);
      end_of_line(csa);
      /* read node descriptor lines */
      s = t = 0;
      for (;;)
      {  read_designator(csa);
         if (strcmp(csa->field, "n") != 0) break;
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "node number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "node number %d out of range", i);
         read_field(csa);
         if (strcmp(csa->field, "s") == 0)
         {  if (s > 0)
               error(csa, "only one source node allowed");
            s = i;
         }
         else if (strcmp(csa->field, "t") == 0)
         {  if (t > 0)
               error(csa, "only one sink node allowed");
            t = i;
         }
         else
            error(csa, "wrong node designator; `s' or `t' expected");
         if (s > 0 && s == t)
            error(csa, "source and sink nodes must be distinct");
         end_of_line(csa);
      }
      if (s == 0)
         error(csa, "source node descriptor missing\n");
      if (t == 0)
         error(csa, "sink node descriptor missing\n");
      if (_s != NULL) *_s = s;
      if (_t != NULL) *_t = t;
      /* read arc descriptor lines */
      for (k = 1; k <= na; k++)
      {  if (k > 1) read_designator(csa);
         if (strcmp(csa->field, "a") != 0)
            error(csa, "wrong line designator; `a' expected");
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "starting node number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "starting node number %d out of range", i);
         read_field(csa);
         if (str2int(csa->field, &j) != 0)
            error(csa, "ending node number missing or invalid");
         if (!(1 <= j && j <= nv))
            error(csa, "ending node number %d out of range", j);
         read_field(csa);
         if (!(str2num(csa->field, &cap) == 0 && cap >= 0.0))
            error(csa, "arc capacity missing or invalid");
         check_int(csa, cap);
         a = glp_add_arc(G, i, j);
         if (a_cap >= 0)
            memcpy((char *)a->data + a_cap, &cap, sizeof(double));
         end_of_line(csa);
      }
      xprintf("%d lines were read\n", csa->count);
done: if (ret) glp_erase_graph(G, G->v_size, G->a_size);
      if (csa->fp != NULL) xfclose(csa->fp);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_write_maxflow - write maximum flow problem data in DIMACS format
*
*  SYNOPSIS
*
*  int glp_write_maxflow(glp_graph *G, int s, int t, int a_cap,
*     const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_write_maxflow writes maximum flow problem data in
*  DIMACS format to a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_write_maxflow(glp_graph *G, int s, int t, int a_cap,
      const char *fname)
{     XFILE *fp;
      glp_vertex *v;
      glp_arc *a;
      int i, count = 0, ret;
      double cap;
      if (!(1 <= s && s <= G->nv))
         xerror("glp_write_maxflow: s = %d; source node number out of r"
            "ange\n", s);
      if (!(1 <= t && t <= G->nv))
         xerror("glp_write_maxflow: t = %d: sink node number out of ran"
            "ge\n", t);
      if (a_cap >= 0 && a_cap > G->a_size - (int)sizeof(double))
         xerror("glp_write_mincost: a_cap = %d; invalid offset\n",
            a_cap);
      xprintf("Writing maximum flow problem data to `%s'...\n",
         fname);
      fp = xfopen(fname, "w");
      if (fp == NULL)
      {  xprintf("Unable to create `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xfprintf(fp, "c %s\n",
         G->name == NULL ? "unknown" : G->name), count++;
      xfprintf(fp, "p max %d %d\n", G->nv, G->na), count++;
      xfprintf(fp, "n %d s\n", s), count++;
      xfprintf(fp, "n %d t\n", t), count++;
      for (i = 1; i <= G->nv; i++)
      {  v = G->v[i];
         for (a = v->out; a != NULL; a = a->t_next)
         {  if (a_cap >= 0)
               memcpy(&cap, (char *)a->data + a_cap, sizeof(double));
            else
               cap = 1.0;
            xfprintf(fp, "a %d %d %.*g\n",
               a->tail->i, a->head->i, DBL_DIG, cap), count++;
         }
      }
      xfprintf(fp, "c eof\n"), count++;
      xfflush(fp);
      if (xferror(fp))
      {  xprintf("Write error on `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xprintf("%d lines were written\n", count);
      ret = 0;
done: if (fp != NULL) xfclose(fp);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_read_asnprob - read assignment problem data in DIMACS format
*
*  SYNOPSIS
*
*  int glp_read_asnprob(glp_graph *G, int v_set, int a_cost,
*     const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_read_asnprob reads assignment problem data in DIMACS
*  format from a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_read_asnprob(glp_graph *G, int v_set, int a_cost, const char
      *fname)
{     struct csa _csa, *csa = &_csa;
      glp_vertex *v;
      glp_arc *a;
      int nv, na, n1, i, j, k, ret = 0;
      double cost;
      char *flag = NULL;
      if (v_set >= 0 && v_set > G->v_size - (int)sizeof(int))
         xerror("glp_read_asnprob: v_set = %d; invalid offset\n",
            v_set);
      if (a_cost >= 0 && a_cost > G->a_size - (int)sizeof(double))
         xerror("glp_read_asnprob: a_cost = %d; invalid offset\n",
            a_cost);
      glp_erase_graph(G, G->v_size, G->a_size);
      if (setjmp(csa->jump))
      {  ret = 1;
         goto done;
      }
      csa->fname = fname;
      csa->fp = NULL;
      csa->count = 0;
      csa->c = '\n';
      csa->field[0] = '\0';
      csa->empty = csa->nonint = 0;
      xprintf("Reading assignment problem data from `%s'...\n", fname);
      csa->fp = xfopen(fname, "r");
      if (csa->fp == NULL)
      {  xprintf("Unable to open `%s' - %s\n", fname, xerrmsg());
         longjmp(csa->jump, 1);
      }
      /* read problem line */
      read_designator(csa);
      if (strcmp(csa->field, "p") != 0)
         error(csa, "problem line missing or invalid");
      read_field(csa);
      if (strcmp(csa->field, "asn") != 0)
         error(csa, "wrong problem designator; `asn' expected");
      read_field(csa);
      if (!(str2int(csa->field, &nv) == 0 && nv >= 0))
         error(csa, "number of nodes missing or invalid");
      read_field(csa);
      if (!(str2int(csa->field, &na) == 0 && na >= 0))
         error(csa, "number of arcs missing or invalid");
      if (nv > 0) glp_add_vertices(G, nv);
      end_of_line(csa);
      /* read node descriptor lines */
      flag = xcalloc(1+nv, sizeof(char));
      memset(&flag[1], 0, nv * sizeof(char));
      n1 = 0;
      for (;;)
      {  read_designator(csa);
         if (strcmp(csa->field, "n") != 0) break;
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "node number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "node number %d out of range", i);
         if (flag[i])
            error(csa, "duplicate descriptor of node %d", i);
         flag[i] = 1, n1++;
         end_of_line(csa);
      }
      xprintf(
         "Assignment problem has %d + %d = %d node%s and %d arc%s\n",
         n1, nv - n1, nv, nv == 1 ? "" : "s", na, na == 1 ? "" : "s");
      if (v_set >= 0)
      {  for (i = 1; i <= nv; i++)
         {  v = G->v[i];
            k = (flag[i] ? 0 : 1);
            memcpy((char *)v->data + v_set, &k, sizeof(int));
         }
      }
      /* read arc descriptor lines */
      for (k = 1; k <= na; k++)
      {  if (k > 1) read_designator(csa);
         if (strcmp(csa->field, "a") != 0)
            error(csa, "wrong line designator; `a' expected");
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "starting node number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "starting node number %d out of range", i);
         if (!flag[i])
            error(csa, "node %d cannot be a starting node", i);
         read_field(csa);
         if (str2int(csa->field, &j) != 0)
            error(csa, "ending node number missing or invalid");
         if (!(1 <= j && j <= nv))
            error(csa, "ending node number %d out of range", j);
         if (flag[j])
            error(csa, "node %d cannot be an ending node", j);
         read_field(csa);
         if (str2num(csa->field, &cost) != 0)
            error(csa, "arc cost missing or invalid");
         check_int(csa, cost);
         a = glp_add_arc(G, i, j);
         if (a_cost >= 0)
            memcpy((char *)a->data + a_cost, &cost, sizeof(double));
         end_of_line(csa);
      }
      xprintf("%d lines were read\n", csa->count);
done: if (ret) glp_erase_graph(G, G->v_size, G->a_size);
      if (csa->fp != NULL) xfclose(csa->fp);
      if (flag != NULL) xfree(flag);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_write_asnprob - write assignment problem data in DIMACS format
*
*  SYNOPSIS
*
*  int glp_write_asnprob(glp_graph *G, int v_set, int a_cost,
*     const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_write_asnprob writes assignment problem data in
*  DIMACS format to a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_write_asnprob(glp_graph *G, int v_set, int a_cost, const char
      *fname)
{     XFILE *fp;
      glp_vertex *v;
      glp_arc *a;
      int i, k, count = 0, ret;
      double cost;
      if (v_set >= 0 && v_set > G->v_size - (int)sizeof(int))
         xerror("glp_write_asnprob: v_set = %d; invalid offset\n",
            v_set);
      if (a_cost >= 0 && a_cost > G->a_size - (int)sizeof(double))
         xerror("glp_write_asnprob: a_cost = %d; invalid offset\n",
            a_cost);
      xprintf("Writing assignment problem data to `%s'...\n", fname);
      fp = xfopen(fname, "w");
      if (fp == NULL)
      {  xprintf("Unable to create `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xfprintf(fp, "c %s\n",
         G->name == NULL ? "unknown" : G->name), count++;
      xfprintf(fp, "p asn %d %d\n", G->nv, G->na), count++;
      for (i = 1; i <= G->nv; i++)
      {  v = G->v[i];
         if (v_set >= 0)
            memcpy(&k, (char *)v->data + v_set, sizeof(int));
         else
            k = (v->out != NULL ? 0 : 1);
         if (k == 0)
            xfprintf(fp, "n %d\n", i), count++;
      }
      for (i = 1; i <= G->nv; i++)
      {  v = G->v[i];
         for (a = v->out; a != NULL; a = a->t_next)
         {  if (a_cost >= 0)
               memcpy(&cost, (char *)a->data + a_cost, sizeof(double));
            else
               cost = 1.0;
            xfprintf(fp, "a %d %d %.*g\n",
               a->tail->i, a->head->i, DBL_DIG, cost), count++;
         }
      }
      xfprintf(fp, "c eof\n"), count++;
      xfflush(fp);
      if (xferror(fp))
      {  xprintf("Write error on `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xprintf("%d lines were written\n", count);
      ret = 0;
done: if (fp != NULL) xfclose(fp);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_read_ccformat - read graph in DIMACS clique/coloring format
*
*  SYNOPSIS
*
*  int glp_read_ccformat(glp_graph *G, int v_wgt, const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_read_ccformat reads an (undirected) graph in DIMACS
*  clique/coloring format from a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_read_ccformat(glp_graph *G, int v_wgt, const char *fname)
{     struct csa _csa, *csa = &_csa;
      glp_vertex *v;
      int i, j, k, nv, ne, ret = 0;
      double w;
      char *flag = NULL;
      if (v_wgt >= 0 && G->v_size - (int)sizeof(double))
         xerror("glp_read_ccformat: v_wgt = %d; invalid offset\n",
            v_wgt);
      glp_erase_graph(G, G->v_size, G->a_size);
      if (setjmp(csa->jump))
      {  ret = 1;
         goto done;
      }
      csa->fname = fname;
      csa->fp = NULL;
      csa->count = 0;
      csa->c = '\n';
      csa->field[0] = '\0';
      csa->empty = csa->nonint = 0;
      xprintf("Reading graph from `%s'...\n", fname);
      csa->fp = xfopen(fname, "r");
      if (csa->fp == NULL)
      {  xprintf("Unable to open `%s' - %s\n", fname, xerrmsg());
         longjmp(csa->jump, 1);
      }
      /* read problem line */
      read_designator(csa);
      if (strcmp(csa->field, "p") != 0)
         error(csa, "problem line missing or invalid");
      read_field(csa);
      if (strcmp(csa->field, "edge") != 0)
         error(csa, "wrong problem designator; `edge' expected");
      read_field(csa);
      if (!(str2int(csa->field, &nv) == 0 && nv >= 0))
         error(csa, "number of vertices missing or invalid");
      read_field(csa);
      if (!(str2int(csa->field, &ne) == 0 && ne >= 0))
         error(csa, "number of edges missing or invalid");
      xprintf("Graph has %d vert%s and %d edge%s\n",
         nv, nv == 1 ? "ex" : "ices", ne, ne == 1 ? "" : "s");
      if (nv > 0) glp_add_vertices(G, nv);
      end_of_line(csa);
      /* read node descriptor lines */
      flag = xcalloc(1+nv, sizeof(char));
      memset(&flag[1], 0, nv * sizeof(char));
      if (v_wgt >= 0)
      {  w = 1.0;
         for (i = 1; i <= nv; i++)
         {  v = G->v[i];
            memcpy((char *)v->data + v_wgt, &w, sizeof(double));
         }
      }
      for (;;)
      {  read_designator(csa);
         if (strcmp(csa->field, "n") != 0) break;
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "vertex number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "vertex number %d out of range", i);
         if (flag[i])
            error(csa, "duplicate descriptor of vertex %d", i);
         read_field(csa);
         if (str2num(csa->field, &w) != 0)
            error(csa, "vertex weight missing or invalid");
         check_int(csa, w);
         if (v_wgt >= 0)
         {  v = G->v[i];
            memcpy((char *)v->data + v_wgt, &w, sizeof(double));
         }
         flag[i] = 1;
         end_of_line(csa);
      }
      xfree(flag), flag = NULL;
      /* read edge descriptor lines */
      for (k = 1; k <= ne; k++)
      {  if (k > 1) read_designator(csa);
         if (strcmp(csa->field, "e") != 0)
            error(csa, "wrong line designator; `e' expected");
         read_field(csa);
         if (str2int(csa->field, &i) != 0)
            error(csa, "first vertex number missing or invalid");
         if (!(1 <= i && i <= nv))
            error(csa, "first vertex number %d out of range", i);
         read_field(csa);
         if (str2int(csa->field, &j) != 0)
            error(csa, "second vertex number missing or invalid");
         if (!(1 <= j && j <= nv))
            error(csa, "second vertex number %d out of range", j);
         glp_add_arc(G, i, j);
         end_of_line(csa);
      }
      xprintf("%d lines were read\n", csa->count);
done: if (ret) glp_erase_graph(G, G->v_size, G->a_size);
      if (csa->fp != NULL) xfclose(csa->fp);
      if (flag != NULL) xfree(flag);
      return ret;
}

/***********************************************************************
*  NAME
*
*  glp_write_ccformat - write graph in DIMACS clique/coloring format
*
*  SYNOPSIS
*
*  int glp_write_ccformat(glp_graph *G, int v_wgt, const char *fname);
*
*  DESCRIPTION
*
*  The routine glp_write_ccformat writes the specified graph in DIMACS
*  clique/coloring format to a text file.
*
*  RETURNS
*
*  If the operation was successful, the routine returns zero. Otherwise
*  it prints an error message and returns non-zero. */

int glp_write_ccformat(glp_graph *G, int v_wgt, const char *fname)
{     XFILE *fp;
      glp_vertex *v;
      glp_arc *e;
      int i, count = 0, ret;
      double w;
      if (v_wgt >= 0 && G->v_size - (int)sizeof(double))
         xerror("glp_write_ccformat: v_wgt = %d; invalid offset\n",
            v_wgt);
      xprintf("Writing graph to `%s'\n", fname);
      fp = xfopen(fname, "w");
      if (fp == NULL)
      {  xprintf("Unable to create `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xfprintf(fp, "c %s\n",
         G->name == NULL ? "unknown" : G->name), count++;
      xfprintf(fp, "p edge %d %d\n", G->nv, G->na), count++;
      if (v_wgt >= 0)
      {  for (i = 1; i <= G->nv; i++)
         {  v = G->v[i];
            memcpy(&w, (char *)v->data + v_wgt, sizeof(double));
            if (w != 1.0)
               xfprintf(fp, "n %d %.*g\n", i, DBL_DIG, w), count++;
         }
      }
      for (i = 1; i <= G->nv; i++)
      {  v = G->v[i];
         for (e = v->out; e != NULL; e = e->t_next)
            xfprintf(fp, "e %d %d\n", e->tail->i, e->head->i), count++;
      }
      xfprintf(fp, "c eof\n"), count++;
      xfflush(fp);
      if (xferror(fp))
      {  xprintf("Write error on `%s' - %s\n", fname, xerrmsg());
         ret = 1;
         goto done;
      }
      xprintf("%d lines were written\n", count);
      ret = 0;
done: if (fp != NULL) xfclose(fp);
      return ret;
}

/* eof */
