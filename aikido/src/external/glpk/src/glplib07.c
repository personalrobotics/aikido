/* glplib07.c (memory allocation) */

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

#include "glplib.h"

#if 1 /* 29/VIII-2008 */
/* some processors need data to be properly aligned; the macro
   align_datasize enlarges the specified size of a data item to provide
   a proper alignment of immediately following data */

#define align_datasize(size) ((((size) + 15) / 16) * 16)
/* 16 bytes is sufficient in both 32- and 64-bit environments
   (8 bytes is not sufficient in 64-bit environment due to jmp_buf) */
#endif

/***********************************************************************
*  NAME
*
*  xmalloc - allocate memory block
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void *xmalloc(int size);
*
*  DESCRIPTION
*
*  The routine xmalloc allocates a memory block of size bytes long.
*
*  Note that being allocated the memory block contains arbitrary data
*  (not binary zeros).
*
*  RETURNS
*
*  The routine xmalloc returns a pointer to the allocated memory block.
*  To free this block the routine xfree (not free!) should be used. */

void *xmalloc(int size)
{     LIBENV *env = lib_link_env();
      LIBMEM *desc;
      int size_of_desc = align_datasize(sizeof(LIBMEM));
      if (size < 1 || size > INT_MAX - size_of_desc)
         xerror("xmalloc: size = %d; invalid parameter\n", size);
      size += size_of_desc;
      if (xlcmp(xlset(size),
          xlsub(env->mem_limit, env->mem_total)) > 0)
         xerror("xmalloc: memory limit exceeded\n");
      if (env->mem_count == INT_MAX)
         xerror("xmalloc: too many memory blocks allocated\n");
      desc = malloc(size);
      if (desc == NULL)
         xerror("xmalloc: no memory available\n");
      memset(desc, '?', size);
      desc->flag = LIB_MEM_FLAG;
      desc->size = size;
      desc->prev = NULL;
      desc->next = env->mem_ptr;
      if (desc->next != NULL) desc->next->prev = desc;
      env->mem_ptr = desc;
      env->mem_count++;
      if (env->mem_cpeak < env->mem_count)
         env->mem_cpeak = env->mem_count;
      env->mem_total = xladd(env->mem_total, xlset(size));
      if (xlcmp(env->mem_tpeak, env->mem_total) < 0)
         env->mem_tpeak = env->mem_total;
      return (void *)((char *)desc + size_of_desc);
}

/***********************************************************************
*  NAME
*
*  xcalloc - allocate memory block
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void *xcalloc(int n, int size);
*
*  DESCRIPTION
*
*  The routine xcalloc allocates a memory block of (n*size) bytes long.
*
*  Note that being allocated the memory block contains arbitrary data
*  (not binary zeros).
*
*  RETURNS
*
*  The routine xcalloc returns a pointer to the allocated memory block.
*  To free this block the routine xfree (not free!) should be used. */

void *xcalloc(int n, int size)
{     if (n < 1)
         xerror("xcalloc: n = %d; invalid parameter\n", n);
      if (size < 1)
         xerror("xcalloc: size = %d; invalid parameter\n", size);
      if (n > INT_MAX / size)
         xerror("xcalloc: n = %d; size = %d; array too big\n", n, size);
      return xmalloc(n * size);
}

/**********************************************************************/

#if 0
void *xrealloc(void *ptr, int size)
{     /* reallocate memory block */
      LIBMEM *desc;
      int size_of_desc = align_datasize(sizeof(LIBMEM));
      int copy_size;
      void *old;
      if (size < 0)
         xerror("xrealloc: size = %d; invalid parameter\n", size);
      if (ptr == NULL)
      {  if (size > 0)
            ptr = xmalloc(size);
         goto done;
      }
      if (size == 0)
      {  xfree(ptr);
         ptr = NULL;
         goto done;
      }
      desc = (void *)((char *)ptr - size_of_desc);
      if (desc->flag != LIB_MEM_FLAG)
         xerror("xrealloc: ptr = %p; invalid pointer\n", ptr);
      copy_size = desc->size - size_of_desc;
      if (copy_size > size) copy_size = size;
      old = ptr;
      ptr = xmalloc(size);
      memcpy(ptr, old, copy_size);
      xfree(old);
done: return ptr;
}
#endif

/***********************************************************************
*  NAME
*
*  xfree - free memory block
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void xfree(void *ptr);
*
*  DESCRIPTION
*
*  The routine xfree frees a memory block pointed to by ptr, which was
*  previuosly allocated by the routine xmalloc or xcalloc. */

void xfree(void *ptr)
{     LIBENV *env = lib_link_env();
      LIBMEM *desc;
      int size_of_desc = align_datasize(sizeof(LIBMEM));
      if (ptr == NULL)
         xerror("xfree: ptr = %p; null pointer\n", ptr);
      desc = (void *)((char *)ptr - size_of_desc);
      if (desc->flag != LIB_MEM_FLAG)
         xerror("xfree: ptr = %p; invalid pointer\n", ptr);
      if (env->mem_count == 0 ||
          xlcmp(env->mem_total, xlset(desc->size)) < 0)
         xerror("xfree: memory allocation error\n");
      if (desc->prev == NULL)
         env->mem_ptr = desc->next;
      else
         desc->prev->next = desc->next;
      if (desc->next == NULL)
         ;
      else
         desc->next->prev = desc->prev;
      env->mem_count--;
      env->mem_total = xlsub(env->mem_total, xlset(desc->size));
      memset(desc, '?', size_of_desc);
      free(desc);
      return;
}

/***********************************************************************
*  NAME
*
*  lib_mem_limit - set memory allocation limit
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void lib_mem_limit(xlong_t limit);
*
*  DESCRIPTION
*
*  The routine lib_mem_limit limits the amount of memory available for
*  dynamic allocation (in GLPK routines) to limit bytes. */

void lib_mem_limit(xlong_t limit)
{     LIBENV *env = lib_link_env();
      env->mem_limit = limit;
      return;
}

/***********************************************************************
*  NAME
*
*  lib_mem_usage - get memory usage information
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void lib_mem_usage(int *count, int *cpeak, xlong_t *total,
*     xlong_t *tpeak);
*
*  DESCRIPTION
*
*  The routine lib_mem_usage reports some information about utilization
*  of the memory by GLPK routines. Information is stored to locations
*  specified by corresponding parameters (see below). Any parameter can
*  be specified as NULL, in which case corresponding information is not
*  stored.
*
*  *count is the number of the memory blocks currently allocated by the
*  routines xmalloc and xcalloc (one call to xmalloc or xcalloc results
*  in allocating one memory block).
*
*  *cpeak is the peak value of *count reached since the initialization
*  of the GLPK library environment.
*
*  *total is the total amount, in bytes, of the memory blocks currently
*  allocated by the routines xmalloc and xcalloc.
*
*  *tpeak is the peak value of *total reached since the initialization
*  of the GLPK library envirionment. */

void lib_mem_usage(int *count, int *cpeak, xlong_t *total,
      xlong_t *tpeak)
{     LIBENV *env = lib_link_env();
      if (count != NULL) *count = env->mem_count;
      if (cpeak != NULL) *cpeak = env->mem_cpeak;
      if (total != NULL) *total = env->mem_total;
      if (tpeak != NULL) *tpeak = env->mem_tpeak;
      return;
}

/* eof */
