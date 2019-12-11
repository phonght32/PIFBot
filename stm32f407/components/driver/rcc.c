/* Includes ------------------------------------------------------------------*/
#include "include/rcc.h"

/* Internal define -----------------------------------------------------------*/


/* Internal typedef ----------------------------------------------------------*/


/* Internal variable ---------------------------------------------------------*/


/* Internal function ---------------------------------------------------------*/


/* External function ---------------------------------------------------------*/
int rcc_set_pll(rcc_pll_t *config)
{
    uint16_t timeout;
    rcc_pll_t tmp;

    /* Read PLL settings */
    rcc_get_pll(&tmp);

    /* Check if structures are equal */
    if (memcmp(config, &tmp, sizeof(rcc_pll_t)) == 0) {
        /* Don't change PLL settings if settings are the same */
        return -1;
    }

    /* Enable HSI clock */
    RCC->CR |= RCC_CR_HSION;

    /* Wait till HSI is ready */
    timeout = 0xFFFF;
    while (!(RCC->CR & RCC_CR_HSIRDY) && timeout--);

    /* Select HSI clock as main clock */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSI;

    /* Disable PLL */
    RCC->CR &= ~RCC_CR_PLLON;

    /* Set PLL settings */
    if (config->PLLM) {
        RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLM_MASK) | ((config->PLLM << RCC_PLLM_POS) & RCC_PLLM_MASK);
    }
    if (config->PLLN) {
        RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLN_MASK) | ((config->PLLN << RCC_PLLN_POS) & RCC_PLLN_MASK);
    }
    if (config->PLLP) {
        RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLP_MASK) | ((((config->PLLP >> 1) - 1) << RCC_PLLP_POS) & RCC_PLLP_MASK);
    }
    if (config->PLLQ) {
        RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLQ_MASK) | ((config->PLLQ << RCC_PLLQ_POS) & RCC_PLLQ_MASK);
    }
    if (config->PLLR) {
        RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLR_MASK) | ((config->PLLR << RCC_PLLR_POS) & RCC_PLLR_MASK);
    }

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    timeout = 0xFFFF;
    while (!rcc_is_ready() && timeout--);

    /* Enable PLL as main clock */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;

    /* Update system core clock variable */
    SystemCoreClockUpdate();

    return 0;
}

int rcc_get_pll(rcc_pll_t *config)
{
    /* Read all PLL settings */
    config->PLLM = (RCC->PLLCFGR & RCC_PLLM_MASK) >> RCC_PLLM_POS;
    config->PLLN = (RCC->PLLCFGR & RCC_PLLN_MASK) >> RCC_PLLN_POS;
    config->PLLP = (((RCC->PLLCFGR & RCC_PLLP_MASK) >> RCC_PLLP_POS) + 1) << 1;
    config->PLLQ = (RCC->PLLCFGR & RCC_PLLQ_MASK) >> RCC_PLLQ_POS;
    config->PLLR = (RCC->PLLCFGR & RCC_PLLR_MASK) >> RCC_PLLR_POS;

    return 0;
}

uint8_t rcc_is_ready(void)
{
    /* Return PLL ready status */
    return (RCC->CR & RCC_CR_PLLRDY);
}
