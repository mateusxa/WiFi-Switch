/*************************************************************************************************************/
/* --- INCLUDES ---------------------------------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_delay.h"
//#include "math.h"

/*************************************************************************************************************/
/* --- DEFINE/MACRO -----------------------------------------------------------------------------------------*/

#define RED_LED_PORT                      GPIOD
#define RED_LED_PIN                       GPIO_PIN_3

#define GREEN_LED_PORT                    GPIOD
#define GREEN_LED_PIN                     GPIO_PIN_1

#define BLUE_LED_PORT                     GPIOC
#define BLUE_LED_PIN                      GPIO_PIN_5


#define RED_LEDSTP_PORT                   GPIOC
#define RED_LEDSTP_PIN                    GPIO_PIN_4

#define GREEN_LEDSTP_PORT                 GPIOC
#define GREEN_LEDSTP_PIN                  GPIO_PIN_3

#define BLUE_LEDSTP_PORT                  GPIOA
#define BLUE_LEDSTP_PIN                   GPIO_PIN_3


#define TIM1_PERIOD                       255
#define TIM2_PERIOD                       255
#define TIM4_PERIOD                       255

/* ----------------------------------------------------------------------------------------------------------*/

#define RED_LED_ON                        GPIO_WriteHigh(RED_LED_PORT, RED_LED_PIN)
#define RED_LED_OFF                       GPIO_WriteLow(RED_LED_PORT, RED_LED_PIN)
#define READ_RED_LED                      GPIO_ReadInputPin(RED_LED_PORT, RED_LED_PIN)

#define GREEN_LED_ON                      GPIO_WriteHigh(GREEN_LED_PORT, GREEN_LED_PIN)
#define GREEN_LED_OFF                     GPIO_WriteLow(GREEN_LED_PORT, GREEN_LED_PIN)
#define READ_GREEN_LED                    GPIO_ReadInputPin(GREEN_LED_PORT, GREEN_LED_PIN)

#define BLUE_LED_ON                       GPIO_WriteHigh(BLUE_LED_PORT, BLUE_LED_PIN)
#define BLUE_LED_OFF                      GPIO_WriteLow(BLUE_LED_PORT, BLUE_LED_PIN)
#define READ_BLUE_LED                     GPIO_ReadInputPin(BLUE_LED_PORT, BLUE_LED_PIN)


#define RED_LEDSTP_ON                     GPIO_WriteHigh(RED_LEDSTP_PORT, RED_LEDSTP_PIN)
#define RED_LEDSTP_OFF                    GPIO_WriteLow(RED_LEDSTP_PORT, RED_LEDSTP_PIN)
#define READ_RED_LEDSTP                   GPIO_ReadInputPin(RED_LEDSTP_PORT, RED_LEDSTP_PIN)

#define GREEN_LEDSTP_ON                   GPIO_WriteHigh(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN)
#define GREEN_LEDSTP_OFF                  GPIO_WriteLow(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN)
#define READ_GREEN_LEDSTP                 GPIO_ReadInputPin(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN)

#define BLUE_LEDSTP_ON                    GPIO_WriteHigh(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN)
#define BLUE_LEDSTP_OFF                   GPIO_WriteLow(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN)
#define READ_BLUE_LEDSTP                  GPIO_ReadInputPin(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN)


/*************************************************************************************************************/
/* --- FUNCTION ---------------------------------------------------------------------------------------------*/

void MCU_init(void);
void OSC_init(void);
void GPIO_init(void);
void TMR1_init(void);
void TMR2_init(void);
void TMR4_init(void);

/*************************************************************************************************************/
/* --- VARIABLES --------------------------------------------------------------------------------------------*/

uint8_t Red_PWM_Value = 127;
uint8_t Green_PWM_Value = 127;
uint16_t Blue_PWM_Value = 127;

/*************************************************************************************************************/
/* --- INTERRUPT HANDLER ------------------------------------------------------------------------------------*/

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
 {  
    if(Red_PWM_Value > 250) RED_LEDSTP_ON;
    else if(Red_PWM_Value < 10) RED_LEDSTP_OFF;
    else{
      if(READ_RED_LEDSTP){                                  // Is Output pin HIGH?
          TIM1_SetCounter(Red_PWM_Value);                      // Set PWM duty cycle low value
          RED_LEDSTP_OFF;                                     // Turn off Output pin
      }else{                                                 // Output pin is LOW
        TIM1_SetCounter(TIM1_PERIOD - Red_PWM_Value);        // Set PWM duty cycle high value
        RED_LEDSTP_ON;                                      // Turn on Output pin
      }
    }
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);                      // Reset Overflow Flag
 }

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
 {  
    if(Green_PWM_Value > 250) GREEN_LEDSTP_ON;
    else if(Green_PWM_Value < 10) GREEN_LEDSTP_OFF;
    else{
      if(READ_GREEN_LEDSTP){                                  // Is Output pin HIGH?
        TIM2_SetCounter(Green_PWM_Value);                      // Set PWM duty cycle low value
        GREEN_LEDSTP_OFF;                                     // Turn off Output pin
      }else{                                                 // Output pin is LOW
        TIM2_SetCounter(TIM2_PERIOD - Green_PWM_Value);        // Set PWM duty cycle high value
        GREEN_LEDSTP_ON;                                      // Turn on Output pin
      }
    }
    TIM2_ClearFlag(TIM2_FLAG_UPDATE);                      // Reset Overflow Flag
 }

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
 {  
    if(Blue_PWM_Value > 250) BLUE_LEDSTP_ON;
    else if(Blue_PWM_Value < 10) BLUE_LEDSTP_OFF;
    else{
      if(READ_BLUE_LEDSTP){                                  // Is Output pin HIGH?
        TIM4_SetCounter(Blue_PWM_Value);                      // Set PWM duty cycle low value
        BLUE_LEDSTP_OFF;                                     // Turn off Output pin
      }else{                                                 // Output pin is LOW
        TIM4_SetCounter(TIM4_PERIOD - Blue_PWM_Value);        // Set PWM duty cycle high value
        BLUE_LEDSTP_ON;                                      // Turn on Output pin
      }
    }
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);                      // Reset Overflow Flag
 }


/*************************************************************************************************************/
/* --- MAIN LOOP --------------------------------------------------------------------------------------------*/

int main( void )
{

  MCU_init();

  /* --- LOOP --- */
  while(1){
    // for(int i = 0; i < TIM4_PERIOD; i += 5){
    //     Red_PWM_Value = i; // 0.784*exp(0.023*i);
    //     Green_PWM_Value = i; // 0.784*exp(0.023*i);
    //     Blue_PWM_Value = i; // 0.784*exp(0.023*i);
    //   Delay_ms(50);
    // }

    RED_LED_ON;
    Delay_ms(500);
    RED_LED_OFF;
    Delay_ms(500);
    GREEN_LED_ON;
    Delay_ms(500);
    GREEN_LED_OFF;
    Delay_ms(500);
    BLUE_LED_ON;
    Delay_ms(500);
    BLUE_LED_OFF;
    Delay_ms(500);

  }
}

/*************************************************************************************************************/
/* --- FUNCTION ---------------------------------------------------------------------------------------------*/

/**
  * @brief  Microcontroller initialization function
  */
void MCU_init(void){
    OSC_init();
    GPIO_init();
    Delay_100us();
    TMR1_init();
    TMR2_init();
    TMR4_init();
    Delay_100us();
    enableInterrupts();
}

/**
  * @brief  High Frequency Internal Oscillator configuration | Fmaster -> 16 MHz | Fcpu -> 16 MHz
  */
void OSC_init(void){
  	CLK_DeInit();								                                                // Reseta as config. de clock
    CLK_HSECmd(DISABLE);						                                            // External Clock disable
    CLK_HSICmd(ENABLE);							                                            // Internal Clock Enable - 16Mhz
    while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == 0);
    CLK_ClockSwitchCmd(ENABLE);
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV2);                              // 16Mhz / 2 -> Fmaster = 8MHz
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);                                    // 8MHz  -> Fcpu = 8MHz
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE,CLK_CURRENTCLOCKSTATE_ENABLE); 

}

/**
  * @brief  General Ports configuration
  */
void GPIO_init(void){
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);

    GPIO_Init(RED_LED_PORT, RED_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

    GPIO_Init(RED_LEDSTP_PORT, RED_LEDSTP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

}

/**
  * @brief aprox 2 kHz Timer | 16 Prescaler | TIM2_PERIOD
  */
void TMR1_init(void){
    
    TIM1_DeInit();

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
    /* Time base configuration */
    TIM1_TimeBaseInit(16, TIM1_COUNTERMODE_UP, TIM1_PERIOD, 0);
    /* Clear TIM1 update flag */
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    /* Enable TIM1 */
    TIM1_Cmd(ENABLE);    
}

/**
  * @brief aprox 2 kHz Timer | 16 Prescaler | TIM2_PERIOD
  */
void TMR2_init(void){
    
    TIM2_DeInit();

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    /* Time base configuration */
    TIM2_TimeBaseInit(TIM2_PRESCALER_16, TIM2_PERIOD);
    /* Clear TIM2 update flag */
    TIM2_ClearFlag(TIM2_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);    
}

/*************************************************************************************************************/

/**
  * @brief aprox 2 kHz Timer | 16 Prescaler | TIM4_PERIOD
  */
void TMR4_init(void){

    TIM4_DeInit();

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
    /* Time base configuration */
    TIM4_TimeBaseInit(TIM4_PRESCALER_16, TIM4_PERIOD);
    /* Clear TIM4 update flag */
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

    /* Enable TIM4 */
    TIM4_Cmd(ENABLE);

    
}

/*************************************************************************************************************/
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
while(1){}
}
#endif

/*************************************************************************************************************/
