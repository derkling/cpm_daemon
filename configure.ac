AC_PREREQ([2.64])
AC_INIT([cpm_daemon], [0.1.3], [Giuseppe Massari <joe.massanga@gmail.com>])
AC_CONFIG_SRCDIR([src/cpm_daemon.c])
AC_CONFIG_FILES([
 Makefile
 src/Makefile
])
AM_INIT_AUTOMAKE

# Checks for programs.
AC_PATH_PROG(PKG_CONFIG, pkg-config)
AC_MSG_NOTICE([Using pkgconfig files into: $PKG_CONFIG_PATH])
AC_PROG_CC
AM_PROG_CC_C_O

# Checks for libraries.
PKG_CHECK_MODULES(XML, libxml-2.0 >= 2.7.5)
PKG_CHECK_MODULES(SYSFS, libsysfs >= 2.1.0)

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdlib.h string.h sys/socket.h syslog.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_PID_T
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset socket])

AC_OUTPUT
