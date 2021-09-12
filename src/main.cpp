/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f0xx.h"

// ----------------------------------------------------------------------------
//
// Semihosting STM32F0 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate semihosting, display a message on the standard output
// and another message on the standard error.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include "map_0.h"

/*
 *
 * PA0 - ADC in, MAF - ADC_IN0
 * PA7 - PWM - TIM3_CH2
 * PA6 - PWM - TIM3_CH1
 * PA9,10 PB1 - DIP SW
 *
 */

int
main(int argc, char* argv[])
{
  // Show the program parameters (passed via semihosting).
  // Output is via the semihosting output channel.
  trace_dump_args(argc, argv);

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // Send a message to the standard output.
  puts("Standard output message.");

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

  RCC_PCLKConfig(RCC_HCLK_Div2);

  // PWM
  GPIO_InitTypeDef gpioStructure;
  gpioStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
  gpioStructure.GPIO_Mode = GPIO_Mode_AF;
  gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
  gpioStructure.GPIO_OType = GPIO_OType_PP;
  gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpioStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);

  // ADC
  gpioStructure.GPIO_Pin = GPIO_Pin_0;
  gpioStructure.GPIO_Mode = GPIO_Mode_AN;
  gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
  gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpioStructure);

  // DIP SW
  gpioStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  gpioStructure.GPIO_Mode = GPIO_Mode_IN;
  gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
  gpioStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &gpioStructure);
  gpioStructure.GPIO_Pin = GPIO_Pin_1;
  gpioStructure.GPIO_Mode = GPIO_Mode_IN;
  gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
  gpioStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &gpioStructure);


  // setup ADC

  ADC_InitTypeDef adcStructure;

  ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4);

  adcStructure.ADC_ContinuousConvMode = DISABLE;
  adcStructure.ADC_DataAlign = ADC_DataAlign_Right;
  adcStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  adcStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  adcStructure.ADC_Resolution = ADC_Resolution_12b;
  adcStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;

  ADC_Init(ADC1, &adcStructure);

  uint32_t cal = ADC_GetCalibrationFactor(ADC1);
  printf("cal=%d\n", cal);

  //ADC_TempSensorCmd(ENABLE);
  //ADC_VbatCmd(ENABLE);

  ADC_Cmd(ADC1, ENABLE);

  ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_28_5Cycles);

  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = 1;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 256;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);

  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);



  TIM_OCInitTypeDef outputChannelInit = {0,};
  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
  outputChannelInit.TIM_Pulse = 0;
  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_Low;

  TIM_OC2Init(TIM3, &outputChannelInit);

  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
  outputChannelInit.TIM_Pulse = 0;
  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &outputChannelInit);

  TIM_Cmd(TIM3, ENABLE);

  TIM_SetCompare2(TIM3, 50);
  TIM_SetCompare1(TIM3, 50);

  ADC_StartOfConversion(ADC1);

  // enable interrupt
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = ADC1_IRQn;
  nvicStructure.NVIC_IRQChannelPriority = 0;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  //int i=0;

  // wait for interrupt
  for (;;);

}

extern "C" void ADC1_IRQHandler()
{
	//ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	static int i=0;

	// adc input
	uint16_t val = ADC_GetConversionValue(ADC1);

	/*if (i%10000 == 0) {
			  printf("val=%d\n", val);
			  //ADC_StartOfConversion(ADC1);
	}
	i++;*/

	int pwm = map_0[(val>>4)&0xff];
	// pwm output
	TIM_SetCompare2(TIM3, pwm);
	TIM_SetCompare1(TIM3, pwm);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
