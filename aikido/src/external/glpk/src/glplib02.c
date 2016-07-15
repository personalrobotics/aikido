/* glplib02.c (TLS communication) */

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

/**********************************************************************/
/*                 Platform-independent ISO C version                 */
/**********************************************************************/

static void *tls = NULL;
/* in a re-entrant version of the package this variable must be placed
   in the Thread Local Storage (TLS) */

/***********************************************************************
*  NAME
*
*  lib_set_ptr - store global pointer in TLS
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void lib_set_ptr(void *ptr);
*
*  DESCRIPTION
*
*  The routine lib_set_ptr stores a pointer specified by the parameter
*  ptr in the Thread Local Storage (TLS). */

void lib_set_ptr(void *ptr)
{     tls = ptr;
      return;
}

/***********************************************************************
*  NAME
*
*  lib_get_ptr - retrieve global pointer from TLS
*
*  SYNOPSIS
*
*  #include "glplib.h"
*  void *lib_get_ptr(void);
*
*  RETURNS
*
*  The routine lib_get_ptr returns a pointer previously stored by the
*  routine lib_set_ptr. If the latter has not been called yet, NULL is
*  returned. */

void *lib_get_ptr(void)
{     void *ptr;
      ptr = tls;
      return ptr;
}

/* eof */
