/**
 * @file       reg_util.h
 * @brief      
 * @date       2025/10/10
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
#ifndef _REG_UTIL_H_
#define _REG_UTIL_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* CODE */

/* Register bit manipulation macros */
#define REG_WRITE(reg, val)                     ((reg) = (val))
#define REG_READ(reg)                           ((reg))
#define REG_SET_BIT(reg, pos)                   ((reg) |=  (1u << (pos)))
#define REG_CLR_BIT(reg, pos)                   ((reg) &= ~(1u << (pos)))
#define REG_READ_BIT(reg, pos)                  ((reg) &   (1u << (pos)))
#define REG_CLR_VAL(reg, clrmask, pos)          ((reg) &= ~((clrmask) << (pos)))
#define REG_SET_VAL(reg, val, setmask, pos)     do{\
                                                        REG_CLR_VAL(reg, setmask, pos);\
                                                        ((reg) |= ((val) << (pos)));\
                                                }while(0)

#define REG_READ_VAL(reg, rdmask, pos)          ((REG_READ(reg) >> (pos)) & (rdmask))

#define __weak          __attribute__((weak))

#ifdef __cplusplus
}
#endif
#endif
