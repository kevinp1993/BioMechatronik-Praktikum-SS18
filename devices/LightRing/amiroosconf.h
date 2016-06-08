#ifndef AMIRO_AMIROOSCONF_H_
#define AMIRO_AMIROOSCONF_H_

/**
 * @brief Stack size of the user thread object
 */
#if !defined(USER_THREAD_STACK_SIZE) || defined(__DOXYGEN__)
#define USER_THREAD_STACK_SIZE         1024
#endif

/**
 * @brief Flag to enable/disable als amiroDbgXXX macros
 */
#if !defined(AMIRO_DBG) || defined(__DOXYGEN__)
#define AMIRO_DBG                      TRUE
#endif

#endif // AMIRO_AMIROOSCONF_H_

