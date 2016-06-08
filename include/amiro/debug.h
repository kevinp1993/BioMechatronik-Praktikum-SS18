#ifndef AMIRO_DEBUG_H_
#define AMIRO_DEBUG_H

#include <amiroosconf.h>
#include <chprintf.h>
#include <chdebug.h>



/**
 * @brief Simple message printing macro
 *
 * @param[in] fmt     the string message to print
 * @param[in] args... the optional list of arguments to insert in the message
 *
 * @todo Make output stream adjustable
 */
#define amiroPrintf(fmt, args...) {                   \
  chprintf((BaseSequentialStream*)&SD1, fmt, ##args); \
}

#if AMIRO_DBG || defined(__DOXYGEN__)

/**
 * @brief Macro to print an error
 *
 * @param[in] c       the condition to be verified to be true
 * @param[in] msg     the string message to print
 * @param[in] args... the optional list of arguments to insert in the message
 */
#define amiroDbgError(c, msg, args...) {                                     \
  if (!(c))                                                                  \
    amiroPrintf("ERROR: " msg "\tAT %s:%s()\n", ##args, __FILE__, __func__); \
}

/**
 * @brief Macro to print a warning
 *
 * @param[in] c       the condition to be verified to be true
 * @param[in] msg     the text message
 * @param[in] args... the optional list of arguments to insert in the message
 */
#define amiroDbgWarning(c, msg, args...) {                                      \
  if (!(c))                                                                     \
    amiroPrintf("WARNING: " msg "\tAT: %s:%s()\n", ##args, __FILE__, __func__); \
}

/**
 * @brief Macro to print a note
 *
 * @param[in] msg     the text message
 * @param[in] args... the optional list of arguments to insert in the message
 */
#define amiroDbgNote(msg, args...) {                                       \
  amiroPrintf("NOTE: " msg "\tAT: %s:%s()\n", ##args, __FILE__, __func__); \
}

/**
 * @brief Macro to print a todo
 *
 * @param[in] msg     the text message
 * @param[in] args... the optional list of arguments to insert in the message
 */
#define amiroDbgTodo(msg, args...) {                                       \
  amiroPrintf("TODO: " msg "\tAT: %s:%s()\n", ##args, __FILE__, __func__); \
}

#else

/*
 * empty macro definitions, if AMIRO_DBG is FALSE
 */
#define amiroDbgError(c, msg, args...)
#define amiroDbgWarning(c, msg, args...)
#define amiroDbgNote(msg, args...)
#define amiroDbgTodo(msg, args...)

#endif


#endif // AMIRO_DEBUG_H_

