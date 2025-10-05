/**
 * @file       common.h
 * @brief      
 * @date       2025/10/02
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/

#ifndef _COMMON_H_
#define _COMMON_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include "stm32f407xx.h"

/* CODE */
#define DEBUG

#if defined(DEBUG)
#define TRACE_INFO(format, ...)          printf("%s(%d) : " format, __FUNCTION__,  __LINE__, ##__VA_ARGS__)
#define ERROR(format, ...)              printf("%s(%d) : " format, __FUNCTION__,  __LINE__, ##__VA_ARGS__)
#else
#define TRACE_LOG(format, ...) 
#define ERROR(format, ...)
#endif // DEBUG

#ifdef __cplusplus
}
#endif
#endif
