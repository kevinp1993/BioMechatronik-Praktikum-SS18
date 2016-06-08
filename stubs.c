#include <errno.h>
#undef errno
extern int errno;

/* libc stub */
__attribute__((used)) void *__dso_handle = 0;

/* libc stub */
__attribute__((used)) int _getpid(void) {
  return 1;
}

/* libc stub */
__attribute__((used)) void _exit(int i) {
  (void)i;
  while(1);
}

/* libc stub */

__attribute__((used)) int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}
