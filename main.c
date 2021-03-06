/*************************************************************************************************************/
/* --- INCLUDES ---------------------------------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_delay.h"
//#include "math.h"



/*************************************************************************************************************/
/* --- DEFINE/MACRO -----------------------------------------------------------------------------------------*/

#define RED_BUTTON_PORT                   GPIOA
#define RED_BUTTON_PIN                    GPIO_PIN_1

#define GREEN_BUTTON_PORT                 GPIOD
#define GREEN_BUTTON_PIN                  GPIO_PIN_4

#define BLUE_BUTTON_PORT                  GPIOB
#define BLUE_BUTTON_PIN                   GPIO_PIN_4


#define INCREASE_BUTTON_PORT              GPIOB
#define INCREASE_BUTTON_PIN               GPIO_PIN_5

#define DECREASE_BUTTON_PORT              GPIOC
#define DECREASE_BUTTON_PIN               GPIO_PIN_6

#define SELECT_BUTTON_PORT                GPIOC
#define SELECT_BUTTON_PIN                 GPIO_PIN_7


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


#define LIGHT_BULB_PORT                   GPIOD
#define LIGHT_BULB_PIN                    GPIO_PIN_2


#define TIM1_PERIOD                       156
#define TIM2_PERIOD                       156
#define TIM4_PERIOD                       156

/* ----------------------------------------------------------------------------------------------------------*/

#define READ_RED_BUTTON                   GPIO_ReadInputPin(RED_BUTTON_PORT, RED_BUTTON_PIN)
#define READ_GREEN_BUTTON                 GPIO_ReadInputPin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN)
#define READ_BLUE_BUTTON                  GPIO_ReadInputPin(BLUE_BUTTON_PORT, BLUE_BUTTON_PIN)

#define READ_INCREASE_BUTTON              GPIO_ReadInputPin(INCREASE_BUTTON_PORT, INCREASE_BUTTON_PIN)
#define READ_DECREASE_BUTTON              GPIO_ReadInputPin(DECREASE_BUTTON_PORT, DECREASE_BUTTON_PIN)
#define READ_SELECT_BUTTON                GPIO_ReadInputPin(SELECT_BUTTON_PORT, SELECT_BUTTON_PIN)

#define RED_LED_ON                        GPIO_WriteHigh(RED_LED_PORT, RED_LED_PIN)
#define RED_LED_OFF                       GPIO_WriteLow(RED_LED_PORT, RED_LED_PIN)
#define READ_RED_LED                      GPIO_ReadInputPin(RED_LED_PORT, RED_LED_PIN)

#define GREEN_LED_ON                      GPIO_WriteHigh(GREEN_LED_PORT, GREEN_LED_PIN)
#define GREEN_LED_OFF                     GPIO_WriteLow(GREEN_LED_PORT, GREEN_LED_PIN)
#define READ_GREEN_LED                    GPIO_ReadInputPin(GREEN_LED_PORT, GREEN_LED_PIN)

#define BLUE_LED_ON                       GPIO_WriteHigh(BLUE_LED_PORT, BLUE_LED_PIN)
#define BLUE_LED_OFF                      GPIO_WriteLow(BLUE_LED_PORT, BLUE_LED_PIN)
#define READ_BLUE_LED                     GPIO_ReadInputPin(BLUE_LED_PORT, BLUE_LED_PIN)

#define LIGHT_BULB_ON                     GPIO_WriteHigh(LIGHT_BULB_PORT, LIGHT_BULB_PIN)
#define LIGHT_BULB_OFF                    GPIO_WriteLow(LIGHT_BULB_PORT, LIGHT_BULB_PIN)
#define READ_LIGHT_BULB                   GPIO_ReadInputPin(LIGHT_BULB_PORT, LIGHT_BULB_PIN)


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
void INT_init(void);
void TMR1_init(void);
void TMR2_init(void);
void TMR4_init(void);

/*************************************************************************************************************/
/* --- VARIABLES --------------------------------------------------------------------------------------------*/

uint8_t Red_PWM_Value = 1;
uint8_t Green_PWM_Value = 1;
uint16_t Blue_PWM_Value = 1;

bool Red_Mode_Selected  = FALSE;
bool Green_Mode_Selected  = FALSE;
bool Blue_Mode_Selected  = FALSE;
bool Increase_Button_Pressed = FALSE;
bool Decrease_Button_Pressed = FALSE;


/*************************************************************************************************************/
/* --- INTERRUPT HANDLER ------------------------------------------------------------------------------------*/

INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3){
    if(!READ_RED_BUTTON){
      if(Red_Mode_Selected) Red_Mode_Selected = FALSE;
      else Red_Mode_Selected = TRUE;
    }
}

INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4){
    if(!READ_BLUE_BUTTON){
      if(Blue_Mode_Selected) Blue_Mode_Selected = FALSE;
      else Blue_Mode_Selected = TRUE;
    }

    if(!READ_INCREASE_BUTTON){
      if(Red_Mode_Selected) Red_PWM_Value = Red_PWM_Value + 5;
      if(Green_Mode_Selected) Green_PWM_Value = Green_PWM_Value + 5;
      if(Blue_Mode_Selected){
        if(Blue_PWM_Value + 5 > TIM4_PERIOD) Blue_PWM_Value = TIM4_PERIOD - 1;
        else Blue_PWM_Value = Blue_PWM_Value + 5;
      } 

      Increase_Button_Pressed = TRUE;
    }


}

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5){

    if(!READ_DECREASE_BUTTON){
      if(Red_Mode_Selected){
        if(Red_PWM_Value - 5 < 5) 
        Red_PWM_Value = Red_PWM_Value - 5;
      } 
      if(Green_Mode_Selected) Green_PWM_Value = Green_PWM_Value - 5;
      if(Blue_Mode_Selected){
        if(Blue_PWM_Value - 5 < 5 || Blue_PWM_Value - 5 > TIM4_PERIOD) Blue_PWM_Value = 1;
        else Blue_PWM_Value = Blue_PWM_Value - 5;
      } 

      Decrease_Button_Pressed = TRUE;
    }

    if(!READ_SELECT_BUTTON){
      if(READ_LIGHT_BULB){
        if(Red_PWM_Value == 255 && 
           Green_PWM_Value == 255 &&
           Blue_PWM_Value == 255){

              Red_PWM_Value = 0; 
              Green_PWM_Value = 0;
              Blue_PWM_Value = 0; 
           }
        LIGHT_BULB_OFF;
      }else{
        LIGHT_BULB_ON;  
        Red_PWM_Value = 255; 
        Green_PWM_Value = 255;
        Blue_PWM_Value = 255; 
      }
      
    }


}

INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6){
    if(!READ_GREEN_BUTTON){
      if(Green_Mode_Selected) Green_Mode_Selected = FALSE;
      else Green_Mode_Selected = TRUE;
    }
}

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
 {  
    if(Red_PWM_Value > 250) RED_LEDSTP_ON;
    else if(Red_PWM_Value < 5) RED_LEDSTP_OFF;
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
    else if(Green_PWM_Value < 5) GREEN_LEDSTP_OFF;
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
    if(Blue_PWM_Value > TIM4_PERIOD - 2) BLUE_LEDSTP_ON;
    else if(Blue_PWM_Value < 2) BLUE_LEDSTP_OFF;
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

    if(Red_Mode_Selected) RED_LED_ON;
    else if(!Red_Mode_Selected) RED_LED_OFF;
    
    if(Green_Mode_Selected) GREEN_LED_ON;
    else if(!Green_Mode_Selected) GREEN_LED_OFF;

    if(Blue_Mode_Selected) BLUE_LED_ON;
    else if(!Blue_Mode_Selected) BLUE_LED_OFF;

    if(Blue_Mode_Selected) BLUE_LED_ON;
    else if(!Blue_Mode_Selected) BLUE_LED_OFF;
    /*
    if(Increase_Button_Pressed){
      if(!READ_INCREASE_BUTTON){
        if(Red_Mode_Selected) Red_PWM_Value = Red_PWM_Value + 5;
        if(Green_Mode_Selected) Green_PWM_Value = Green_PWM_Value + 5;
        if(Blue_Mode_Selected) Blue_PWM_Value = Blue_PWM_Value + 5;
      }else Increase_Button_Pressed = FALSE;
      Delay_ms(500);
    }

    if(Decrease_Button_Pressed){
      if(!READ_DECREASE_BUTTON){
        if(Red_Mode_Selected) Red_PWM_Value = Red_PWM_Value - 5;
        if(Green_Mode_Selected) Green_PWM_Value = Green_PWM_Value - 5;
        if(Blue_Mode_Selected) Blue_PWM_Value = Blue_PWM_Value - 5;
      }else Decrease_Button_Pressed = FALSE;
      Delay_ms(500);
    }

    */
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
    INT_init();
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

    GPIO_Init(LIGHT_BULB_PORT, LIGHT_BULB_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

    GPIO_Init(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_MODE_IN_PU_IT);
    GPIO_Init(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_MODE_IN_PU_IT);
    GPIO_Init(BLUE_BUTTON_PORT, BLUE_BUTTON_PIN, GPIO_MODE_IN_PU_IT);

    GPIO_Init(INCREASE_BUTTON_PORT, INCREASE_BUTTON_PIN, GPIO_MODE_IN_PU_IT);
    GPIO_Init(DECREASE_BUTTON_PORT, DECREASE_BUTTON_PIN, GPIO_MODE_IN_PU_IT);
    GPIO_Init(SELECT_BUTTON_PORT, SELECT_BUTTON_PIN, GPIO_MODE_IN_PU_IT);

}

/**
  * @brief  General Ports configuration
  */
void INT_init(void){
  
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_FALL_ONLY);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTA, ITC_PRIORITYLEVEL_1);

  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTB, ITC_PRIORITYLEVEL_1);

  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_FALL_ONLY);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTC, ITC_PRIORITYLEVEL_1);

  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTD, ITC_PRIORITYLEVEL_1);
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
    TIM4_TimeBaseInit(TIM4_PRESCALER_64, TIM4_PERIOD);
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
