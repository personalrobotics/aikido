dnl Process this file with autoconf to produce a configure script

AC_INIT([GLPK], [4.39], [bug-glpk@gnu.org])

AC_CONFIG_SRCDIR([include/glpk.h])

AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE

AC_CONFIG_HEADERS([config.h])

AC_ARG_WITH(gmp,
AC_HELP_STRING([--with-gmp],
   [use GNU MP bignum library [[default=no]]]),
   [case $withval in
      yes | no) ;;
      *) AC_MSG_ERROR([invalid value `$withval' for --with-gmp]);;
      esac],
   [with_gmp=no])

AC_ARG_WITH(zlib,
AC_HELP_STRING([--with-zlib],
   [use zlib data compression library [[default=no]]]),
   [case $withval in
      yes | no) ;;
      *) AC_MSG_ERROR([invalid value `$withval' for --with-zlib]);;
      esac],
   [with_zlib=no])

AC_ARG_ENABLE(dl,
AC_HELP_STRING([--enable-dl],
   [enable shared library support [[default=no]]]),
   [case $enableval in
      yes | ltdl | dlfcn | no) ;;
      *) AC_MSG_ERROR([invalid value `$enableval' for --enable-dl]);;
      esac],
   [enable_dl=no])

AC_ARG_ENABLE(odbc,
AC_HELP_STRING([--enable-odbc],
   [enable MathProg ODBC support [[default=no]]]),
   [case $enableval in
      yes | unix | no) ;;
      *) AC_MSG_ERROR([invalid value `$enableval' for --enable-odbc]);;
      esac],
   [enable_odbc=no])

AC_ARG_ENABLE(mysql,
AC_HELP_STRING([--enable-mysql],
   [enable MathProg MySQL support [[default=no]]]),
   [case $enableval in
      yes | no) ;;
      *) AC_MSG_ERROR([invalid value `$enableval' for --enable-mysql]);;
      esac],
   [enable_mysql=no])

dnl Disable unnecessary libtool tests
define([AC_LIBTOOL_LANG_CXX_CONFIG], [:])
define([AC_LIBTOOL_LANG_F77_CONFIG], [:])
define([AC_LIBTOOL_LANG_GCJ_CONFIG], [:])

dnl Check for programs
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl Check for math library
AC_CHECK_LIB([m], [exp])

dnl Check for vsnprintf function
AC_CHECK_FUNCS([vsnprintf])

AC_MSG_CHECKING([whether to use GNU MP bignum library])
if test "$with_gmp" = "yes"; then
   AC_MSG_RESULT([yes])
   AC_DEFINE([HAVE_GMP], [1], [N/A])
   LIBS="-lgmp $LIBS"
else
   AC_MSG_RESULT([no])
fi

AC_MSG_CHECKING([whether to use zlib data compression library])
if test "$with_zlib" = "yes"; then
   AC_MSG_RESULT([yes])
   AC_DEFINE([HAVE_ZLIB], [1], [N/A])
   LIBS="-lz $LIBS"
else
   AC_MSG_RESULT([no])
fi

AC_MSG_CHECKING([whether to enable shared library support])
if test "$enable_dl" = "yes"; then
   AC_MSG_RESULT([ltdl])
   AC_DEFINE([HAVE_LTDL], [1], [N/A])
   LIBS="-lltdl $LIBS"
elif test "$enable_dl" = "ltdl"; then
   AC_MSG_RESULT([ltdl])
   AC_DEFINE([HAVE_LTDL], [1], [N/A])
   LIBS="-lltdl $LIBS"
elif test "$enable_dl" = "dlfcn"; then
   AC_MSG_RESULT([dlfcn])
   AC_DEFINE([HAVE_DLFCN], [1], [N/A])
else
   AC_MSG_RESULT([no])
fi

AC_MSG_CHECKING([whether to enable MathProg ODBC support])
if test "$enable_odbc" = "yes"; then
   if test "$enable_dl" = "no"; then
      AC_MSG_ERROR([--enable-odbc requires --enable-dl])
   fi
   AC_MSG_RESULT([yes])
   AC_DEFINE([ODBC_DLNAME], ["libiodbc.so"], [N/A])
elif test "$enable_odbc" = "unix"; then
   if test "$enable_dl" = "no"; then
      AC_MSG_ERROR([--enable-odbc requires --enable-dl])
   fi
   AC_MSG_RESULT([unix])
   AC_DEFINE([ODBC_DLNAME], ["libodbc.so"], [N/A])
else
   AC_MSG_RESULT([no])
fi

AC_MSG_CHECKING([whether to enable MathProg MySQL support])
if test "$enable_mysql" = "yes"; then
   if test "$enable_dl" = "no"; then
      AC_MSG_ERROR([--enable-mysql requires --enable-dl])
   fi
   AC_MSG_RESULT([yes])
   CPPFLAGS="-I/usr/include/mysql $CPPFLAGS"
   AC_DEFINE([MYSQL_DLNAME], ["libmysqlclient.so"], [N/A])
else
   AC_MSG_RESULT([no])
fi

AC_CONFIG_FILES(
   [include/Makefile src/Makefile examples/Makefile Makefile])
AC_OUTPUT

dnl eof
