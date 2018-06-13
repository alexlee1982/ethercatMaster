#ifndef _COMMON_H
#define	_COMMON_H

#include <stdio.h>
#include <memory.h>
#include <string.h>
#include <stdlib.h>

typedef unsigned char  u8;                   /* 无符号8位整型变量                        */
typedef signed   char  s8;                    /* 有符号8位整型变量                        */
typedef unsigned short u16;                  /* 无符号16位整型变量                       */
typedef signed   short s16;                   /* 有符号16位整型变量                       */
typedef unsigned int   u32;                  /* 无符号32位整型变量                       */
typedef signed   int   s32;                   /* 有符号32位整型变量                       */
typedef signed   long long s64;                   /* 有符号64位整型变量                       */
typedef unsigned long long u64;                   /* 有符号64位整型变量                       */
typedef float          f32;                    /* 单精度浮点数（32位长度）                 */
typedef double         f64;                    /* 双精度浮点数（64位长度）                 */

#define MAX_S16	32767
#define MIN_S16	(-32768)
#define MAX_S32	2147483647
#define MIN_S32	(-2147483647-1)

#define MIN(a,b)	((a)<(b)?(a):(b))
#define MAX(a,b)	((a)>(b)?(a):(b))

//布朗类型
#ifndef	__cplusplus
typedef unsigned int bool;				
#define	false	0
#define	true	1
#endif

#ifndef B0
#define B0		(0x01ul<<0)
#define B1		(0x01ul<<1)
#define B2		(0x01ul<<2)
#define B3		(0x01ul<<3)
#define B4		(0x01ul<<4)
#define B5		(0x01ul<<5)
#define B6		(0x01ul<<6)
#define B7		(0x01ul<<7)
#define B8		(0x01ul<<8)
#define B9		(0x01ul<<9)
#define B10		(0x01ul<<10)
#define B11		(0x01ul<<11)
#define B12		(0x01ul<<12)
#define B13		(0x01ul<<13)
#define B14		(0x01ul<<14)
#define B15		(0x01ul<<15)
#define B16		(0x01ul<<16)
#define B17		(0x01ul<<17)
#define B18		(0x01ul<<18)
#define B19		(0x01ul<<19)
#define B20		(0x01ul<<20)
#define B21		(0x01ul<<21)
#define B22		(0x01ul<<22)
#define B23		(0x01ul<<23)
#define B24		(0x01ul<<24)
#define B25		(0x01ul<<25)
#define B26		(0x01ul<<26)
#define B27		(0x01ul<<27)
#define B28		(0x01ul<<28)
#define B29		(0x01ul<<29)
#define B30		(0x01ul<<30)
#define B31		(0x01ul<<31)
#endif

#ifndef	NULL
#define NULL	0
#endif

#ifndef INLINE
#define INLINE	__inline
#endif

// 打印错误信息
//#define PRINT_ERROR	printf
#define PRINT_ERROR		printf("ERROR: File=%s,Line=%d,Fun=%s: ",__FILE__,__LINE__,__FUNCTION__);printf
#define PRINT_DBG		printf("DEBUG: File=%s,Line=%d,Fun=%s: ",__FILE__,__LINE__,__FUNCTION__);printf

#if 1
#include <assert.h>
#define ASSERT	assert
#else
#endif

#endif
