/* SPDX-License-Identifier: GPL-2.0 */
/*
 * compat stdarg.h - the 4.14 vendor gen4m gl_kal.c does #include <stdarg.h>
 * expecting the toolchain/freestanding header. Mainline kernels (5.15+) provide
 * the kernel's own <linux/stdarg.h> and forbid the libc one. Redirect.
 * Reached via the compat/include -I path that is on every gen4m TU's cmdline.
 */
#ifndef _COMPAT_STDARG_H
#define _COMPAT_STDARG_H
#include <linux/stdarg.h>
#endif
