#ifndef RCC_H_
#define RCC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

#include "stdint.h"

/* PLL configuration */
#define RCC_PLLM_MASK    ((uint32_t)0x0000003F)
#define RCC_PLLM_POS     0
#define RCC_PLLN_MASK    ((uint32_t)0x00007FC0)
#define RCC_PLLN_POS     6
#define RCC_PLLP_MASK    ((uint32_t)0x00030000)
#define RCC_PLLP_POS     16
#define RCC_PLLQ_MASK    ((uint32_t)0x0F000000)
#define RCC_PLLQ_POS     24
#define RCC_PLLR_MASK    ((uint32_t)0x70000000)
#define RCC_PLLR_POS     28

/**
 * @brief  PLL structure with settings for read and write operations 
 */
typedef struct {
    uint16_t PLLM; /*!< PLL M parameter. This value can be between 2 and 63.    Use 0 if you don't want to change parameter. */
    uint16_t PLLN; /*!< PLL N parameter. This value can be between 192 and 432. Use 0 if you don't want to change parameter. */ 
    uint16_t PLLP; /*!< PLL P parameter. This value can be 2, 4, 6 or 8.        Use 0 if you don't want to change parameter. */
    uint16_t PLLQ; /*!< PLL Q parameter. This value can be between 2 and 15.    Use 0 if you don't want to change parameter. */
    uint16_t PLLR; /*!< PLL R parameter. This value can be between 2 and 7 and is only available for STM32F446 devices. 
                           Use 0 if you don't want to change parameter. */
} rcc_pll_t;

int rcc_set_pll(rcc_pll_t *config);
int rcc_get_pll(rcc_pll_t *config);
uint8_t rcc_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* RCC_H_ */