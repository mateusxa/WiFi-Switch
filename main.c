/*************************************************************************************************************/
/* --- INCLUDES ---------------------------------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_delay.h"

/*************************************************************************************************************/
/* --- DEFINE/MACRO -----------------------------------------------------------------------------------------*/

#define RED_LEDSTP_PORT                   GPIOC
#define RED_LEDSTP_PIN                    GPIO_PIN_4

#define GREEN_LEDSTP_PORT                 GPIOC
#define GREEN_LEDSTP_PIN                  GPIO_PIN_3

#define BLUE_LEDSTP_PORT                  GPIOA
#define BLUE_LEDSTP_PIN                   GPIO_PIN_3

/* ----------------------------------------------------------------------------------------------------------*/

#define RED_LEDSTP_ON                     GPIO_WriteHigh(RED_LEDSTP_PORT, RED_LEDSTP_PIN)
#define RED_LEDSTP_OFF                    GPIO_WriteLow(RED_LEDSTP_PORT, RED_LEDSTP_PIN)

#define GREEN_LEDSTP_ON                     GPIO_WriteHigh(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN)
#define GREEN_LEDSTP_OFF                    GPIO_WriteLow(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN)

#define BLUE_LEDSTP_ON                     GPIO_WriteHigh(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN)
#define BLUE_LEDSTP_OFF                    GPIO_WriteLow(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN)


/*************************************************************************************************************/
/* --- FUNCTION ---------------------------------------------------------------------------------------------*/

void MCU_init(void);
void OSC_init(void);
void GPIO_init(void);

/*************************************************************************************************************/
/* --- MAIN LOOP --------------------------------------------------------------------------------------------*/

int main( void )
{

  MCU_init();

  while(1){
    RED_LEDSTP_ON;
    Delay_ms(500);
    RED_LEDSTP_OFF;
    Delay_ms(500);
    GREEN_LEDSTP_ON;
    Delay_ms(500);
    GREEN_LEDSTP_OFF;
    Delay_ms(500);
    BLUE_LEDSTP_ON;
    Delay_ms(500);
    BLUE_LEDSTP_OFF;
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
}

/**
  * @brief  High Frequency Internal Oscillator configuration | Fmaster -> 8 MHz | Fcpu -> 8 MHz
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
  * @brief  Microcontroller initialization function
  */
void GPIO_init(void){
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);

    GPIO_Init(RED_LEDSTP_PORT, RED_LEDSTP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(GREEN_LEDSTP_PORT, GREEN_LEDSTP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(BLUE_LEDSTP_PORT, BLUE_LEDSTP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

}

/*************************************************************************************************************/

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
while(1){}
}
#endif

/*************************************************************************************************************/
