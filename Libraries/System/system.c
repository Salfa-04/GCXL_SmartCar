/**
 ******************************************************************************
 * @file    system.c
 * @author  MCD Application Team
 * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
 ******************************************************************************
 */

/* Includes */
#include "stm32f4xx.h"

#if !defined(HSE_VALUE)
#define HSE_VALUE \
  ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif                 /* HSE_VALUE */

#if !defined(HSI_VALUE)
#define HSI_VALUE \
  ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif                 /* HSI_VALUE */

/**
 * @brief System Clock Configuration
 * @param  None
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as
 * data memory  */
/* #define DATA_IN_ExtSRAM */

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS                     \
  SRAM_BASE /*!< Vector Table base address field. \
                 This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET                            \
  0x00000000U /*!< Vector Table base offset field. \
                   This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS                      \
  FLASH_BASE /*!< Vector Table base address field. \
                  This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET                            \
  0x00000000U /*!< Vector Table base offset field. \
                   This value must be a multiple of 0x200. */
#endif        /* VECT_TAB_SRAM */
#endif        /* USER_VECT_TAB_ADDRESS */
/******************************************************************************/

/* This variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by calling HAL API function HAL_RCC_GetHCLKFreq()
    3) each time HAL_RCC_ClockConfig() is called to configure the system clock
   frequency Note: If you use this function to configure the system clock; then
   there is no need to call the 2 first functions listed above, since
   SystemCoreClock variable is updated automatically.
*/
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0,
                                   1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

#if defined(DATA_IN_ExtSRAM)
static void SystemInit_ExtMemCtl(void);
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
 * @brief  Setup the microcontroller system
 *         Initialize the FPU setting, vector table location and External memory
 *         configuration.
 * @param  None
 * @retval None
 */
void SystemInit(void) {
/* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |=
      ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif

#if defined(DATA_IN_ExtSRAM)
  SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM */

  /* Configure the Vector Table location -------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS |
              VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif                         /* USER_VECT_TAB_ADDRESS */
}

/**
 * @brief  Update SystemCoreClock variable according to Clock Register Values.
 *         The SystemCoreClock variable contains the core clock (HCLK), it can
 *         be used by the user application to setup the SysTick timer or
 * configure other parameters.
 *
 * @param  None
 * @retval None
 */
void SystemCoreClockUpdate(void) {
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp) {
    case 0x00: /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04: /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08: /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (pllsource != 0) {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      } else {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
      SystemCoreClock = pllvco / pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}

#if defined(DATA_IN_ExtSRAM) && defined(STM32F407xx)
/**
 * @brief  Setup the external memory controller.
 *         Called in startup_stm32f4xx.s before jump to main.
 *         This function configures the external memories (SRAM/SDRAM)
 *         This SRAM/SDRAM will be used as program data memory (including heap
 * and stack).
 * @param  None
 * @retval None
 */
void SystemInit_ExtMemCtl(void) {
  __IO uint32_t tmp = 0x00;

  /*-- GPIOs Configuration
   * -----------------------------------------------------*/
  /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR |= 0x00000078;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0] = 0x00CCC0CC;
  GPIOD->AFR[1] = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */
  GPIOD->MODER = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */
  GPIOD->OTYPER = 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOD->PUPDR = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0] = 0xC00CC0CC;
  GPIOE->AFR[1] = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */
  GPIOE->MODER = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */
  GPIOE->OTYPER = 0x00000000;
  /* No pull-up, pull-down for PEx pins */
  GPIOE->PUPDR = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0] = 0x00CCCCCC;
  GPIOF->AFR[1] = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */
  GPIOF->MODER = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */
  GPIOF->OTYPER = 0x00000000;
  /* No pull-up, pull-down for PFx pins */
  GPIOF->PUPDR = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0] = 0x00CCCCCC;
  GPIOG->AFR[1] = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */
  GPIOG->MODER = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */
  GPIOG->OTYPER = 0x00000000;
  /* No pull-up, pull-down for PGx pins */
  GPIOG->PUPDR = 0x00000000;

  /*-- FMC/FSMC Configuration
   * --------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  RCC->AHB3ENR |= 0x00000001;

  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2] = 0x00001011;
  FSMC_Bank1->BTCR[3] = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;

  (void)(tmp);
}
#endif /* DATA_IN_ExtSRAM && STM32F407xx */
