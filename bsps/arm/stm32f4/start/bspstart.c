/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 * Copyright (c) 2022 Duc Doan (dtbpkmte at gmail.com).
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp.h>
#include <bspopts.h>
#include <bsp/io.h>
#include <bsp/irq.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <assert.h>
#include <stm32f4xx.h>
#include <bsp/gpio2.h>

#ifdef STM32F4_FAMILY_F4XXXX

/* Get number of milliseconds elapsed since startup */
uint32_t HAL_GetTick(void)
{
  return rtems_clock_get_ticks_since_boot() *
    rtems_configuration_get_milliseconds_per_tick();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

static rtems_status_code SystemClock_Config(void)
{
    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    uint32_t use_hse = STM32F4_USE_HSE, sys_clk = STM32F4_SYSCLK / 1000000L, hse_clk = STM32F4_HSE_FREQUENCY / 1000000L, src_clk;
    uint32_t sys_clk_src;
    uint32_t flash_latency = FLASH_LATENCY_0;
    uint32_t pll_m = 0, pll_n = 0, pll_p = 0, pll_q = 0;
    uint32_t apbpre1 = 0, apbpre2 = 0;

    if (sys_clk == 16 && hse_clk != 16) {
        sys_clk_src = RCC_SYSCLKSOURCE_HSI;
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        flash_latency = FLASH_LATENCY_0;
    } else if (sys_clk == hse_clk) {
        sys_clk_src = RCC_SYSCLKSOURCE_HSE;
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        flash_latency = FLASH_LATENCY_0;
    } else {
        sys_clk_src = RCC_SYSCLKSOURCE_PLLCLK;
        if (sys_clk > 180) {
            return RTEMS_INVALID_NUMBER;
        } else if (sys_clk >= 96) {
            pll_n = sys_clk << 1;
            pll_p = RCC_PLLP_DIV2;
        } else if (sys_clk >= 48) {
            pll_n = sys_clk << 2;
            pll_p = RCC_PLLP_DIV4;
        } else if (sys_clk >= 24) {
            pll_n = sys_clk << 3;
            pll_p = RCC_PLLP_DIV8;
        } else {
            return RTEMS_INVALID_NUMBER;
        }

        if (hse_clk == 0 || use_hse == 0) {
            src_clk = 16;
            use_hse = 0;
        } else {
            src_clk = hse_clk;
        }
        
        pll_m = src_clk;

        pll_q = ((long) (src_clk * pll_n)) / pll_m / 48;
        if (pll_q < 2) {
            pll_q = 2;
        }

        /* APB1 prescaler, APB1 clock must be < 42MHz */
        apbpre1 = (sys_clk * 100) / 42;

        if ( apbpre1 <= 100 ) {
            apbpre1 = RCC_HCLK_DIV1;
        } else if (apbpre1 <= 200) {
            apbpre1 = RCC_HCLK_DIV2;
        } else if (apbpre1 <= 400) {
            apbpre1 = RCC_HCLK_DIV4;
        } else if (apbpre1 <= 800) {
            apbpre1 = RCC_HCLK_DIV8;
        } else if (apbpre1) {
            apbpre1 = RCC_HCLK_DIV16;
        }

        /* APB2 prescaler, APB2 clock must be < 84MHz */
        apbpre2 = (sys_clk * 100) / 84;

        if (apbpre2 <= 100) {
            apbpre2 = RCC_HCLK_DIV1;
        } else if (apbpre2 <= 200) {
            apbpre2 = RCC_HCLK_DIV2;
        } else if (apbpre2 <= 400) {
            apbpre2 = RCC_HCLK_DIV4;
        } else if (apbpre2 <= 800) {
            apbpre2 = RCC_HCLK_DIV8;
        } else {
            apbpre2 = RCC_HCLK_DIV16;
        }

        if (use_hse) {
            RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
            RCC_OscInitStruct.HSEState = RCC_HSE_ON;
            RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        } else {
            RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
            RCC_OscInitStruct.HSIState = RCC_HSI_ON;
            RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
            RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        }
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLM = pll_m;
        RCC_OscInitStruct.PLL.PLLN = pll_n;
        RCC_OscInitStruct.PLL.PLLP = pll_p;
        RCC_OscInitStruct.PLL.PLLQ = pll_q;
        flash_latency = FLASH_LATENCY_5;
    }

    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (status != HAL_OK)
    {
//            Error_Handler();
        return RTEMS_UNSATISFIED;
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = sys_clk_src;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = apbpre1;
    RCC_ClkInitStruct.APB2CLKDivider = apbpre2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency) != HAL_OK)
    {
//        Error_Handler();
        return RTEMS_UNSATISFIED;
    }

    return RTEMS_SUCCESSFUL;
}

static void init_main_osc( void )
{
    HAL_Init();
    rtems_status_code status = rtems_gpio_initialize();
    if (status != RTEMS_SUCCESSFUL) {
        Error_Handler();
    }
    status = SystemClock_Config();   
    if (status != RTEMS_SUCCESSFUL) {
        Error_Handler();
    }
}

#endif /* STM32F4_FAMILY_F4XXXX */

#ifdef STM32F4_FAMILY_F10XXX

static void init_main_osc( void )
{

}

#endif /* STM32F4_FAMILY_F10XXX */

void bsp_start( void )
{
    init_main_osc();

    bsp_interrupt_initialize();
}
