#ifndef __STM32_ASSERT_H
#define __STM32_ASSERT_H

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#define assert_param(expr) \
  ((expr) ? (void)0U : assert_failed((uint8_t*)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32_ASSERT_H */
