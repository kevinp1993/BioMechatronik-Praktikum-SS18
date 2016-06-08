/*
 * util.h
 *
 *  Created on: Jun 18, 2014
 *      Author: Robert ABEL
 */

#ifndef UTIL_H_
#define UTIL_H_

#define ARRAY_SIZE(_X_) (sizeof(_X_) / sizeof((_X_)[0]))

#define MEMBER_SIZE(_TYPE_, _MEMBER_) sizeof(((_TYPE_ *)0)->_MEMBER_)

#endif /* UTIL_H_ */
