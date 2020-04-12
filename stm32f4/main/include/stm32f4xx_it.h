#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void);

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void);

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void);

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void);

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void);

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void);

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */