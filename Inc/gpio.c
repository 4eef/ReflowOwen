/*!****************************************************************************
* @file     gpio.c
* @author   Storozhenko Roman - D_EL
* @version  V1.0
* @date     20.07.2016
* @brief    gpio driver for stm32L4
*/

/*!****************************************************************************
* Include
*/
#include "gpio.h"

/*!****************************************************************************
* MEMORY
*/
pinMode_type   const pinsMode[] = {
/*0 */  makepin(GPIOC,  0,  floatingInput,                      0,  0),     //MAX31865_MISO
/*1 */  makepin(GPIOC,  1,  outPushPull,                        1,  0),     //MAX31865_CSn
/*2 */  makepin(GPIOC,  2,  outPushPull,                        0,  0),     //MAX31865_CLK
/*3 */  makepin(GPIOC,  3,  outPushPull,                        0,  0),     //MAX31865_MOSI
/*4 */  makepin(GPIOH,  0,  inputWithPullUp,                    0,  0),     //Button_START
/*5 */  makepin(GPIOH,  1,  inputWithPullUp,                    0,  0),     //Button_ON
/*6 */  makepin(GPIOA,  2,  alternateFunctionOutputPushPull,    1,  7),     //USART_TX
/*7 */  makepin(GPIOA,  3,  alternateFunctionOutputPushPull,    1,  7),     //USART_RX
/*8 */  makepin(GPIOA,  4,  outPushPull,                        0,  0),     //LED_POWER
/*9 */  makepin(GPIOA,  5,  outPushPull,                        0,  0),     //LED_READY
/*10*/  makepin(GPIOA,  6,  outPushPull,                        0,  0),     //LED_PROFILE
/*11*/  makepin(GPIOA,  7,  outPushPull,                        0,  0),     //LED_DOOR
/*12*/  makepin(GPIOC,  4,  outPushPull,                        0,  0),     //LED_OVERHEAT
/*13*/  makepin(GPIOC,  5,  outPushPull,                        0,  0),     //LED_END
/*14*/  makepin(GPIOB,  0,  outPushPull,                        0,  0),     //LED_FIRE
/*15*/  makepin(GPIOB,  1,  outPushPull,                        0,  0),     //LED_HEATER
/*16*/  makepin(GPIOB,  10,  floatingInput,                     0,  0),     //PHASE_SYNC
/*17*/  makepin(GPIOB,  11,  outPushPull,                       0,  0),     //TRIAC_EN
};
uint32_t pinNum = sizeof(pinsMode) / sizeof(pinMode_type);

/*!****************************************************************************
* InitAllGpio
*/
void initGpios(void){
//    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN;                      //Включили тактирование AFIO
//#if (JTAG_OFF > 0)
//    AFIO->MAPR      |= AFIO_MAPR_SWJ_CFG_1; //JTAG
//#endif
    pinMode_type *pgpios;
    pinMode_type *pgpiosEnd;
    
    pgpios = (pinMode_type*)pinsMode;
    pgpiosEnd = pgpios + pinNum;
    
    while(pgpios < pgpiosEnd){
        gppin_init(pgpios->p, pgpios->npin, pgpios->mode, pgpios->iniState, pgpios->nAF);
        pgpios++;
    }
}

/*!****************************************************************************
*
*/
void gppin_init(GPIO_TypeDef *port, uint8_t npin, gpioMode_type mode, uint8_t iniState, uint8_t nAF){
    //Clock enable
    if(port == GPIOA)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOAEN;
    if(port == GPIOB)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOBEN;
    if(port == GPIOC)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOCEN;
    if(port == GPIOD)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIODEN;
    if(port == GPIOE)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOEEN;
    if(port == GPIOF)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOFEN;
    if(port == GPIOG)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOGEN;
    if(port == GPIOH)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOHEN;
    //if(port == GPIOO)   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOOEN;
    
    if(iniState != 0){
        port->BSRR = (1<<npin);
    }
    else{
        port->BRR = (1<<npin);
    }
    
    /*
    * Clear bit field
    */
    port->MODER     &= ~(0x03 << (2 * npin));
    port->OTYPER    &= ~(1<<npin);
    port->PUPDR     &= ~(GPIO_PULL_UP << (2*npin));
    
    switch(mode){
        case analogMode:
            port->MODER |= GPIO_ANALOG_MODE << (2*npin);
            break;
  
        case floatingInput:
            break;
            
        case inputWithPullUp:
            port->PUPDR |= GPIO_PULL_UP << (2*npin);
            break;
            
        case inputWithPullDown:
            port->PUPDR |= GPIO_PULL_DOWN << (2*npin);
            break;
            
        case outPushPull:
            port->MODER |= GPIO_GP_OUT << (2*npin);
            port->OTYPER |= GPIO_PUSH_PULL << npin;
            break;
            
        case outOpenDrain:
            port->MODER |= GPIO_GP_OUT << (2*npin);
            port->OTYPER |= GPIO_OPEN_DRAIN << npin;
            break;
            
       case alternateFunctionOutputPushPull:
            port->MODER |= GPIO_AF_MODE << (2*npin);
            port->OTYPER |= GPIO_PUSH_PULL << npin;
            break;
            
        case alternateFunctionOutputOpenDrain:
            port->MODER |= GPIO_AF_MODE << (2*npin);
            port->OTYPER |= GPIO_OPEN_DRAIN << npin;
            break;   
    }
    
    //Set number alternate function
    port->AFR[npin / 8] = nAF << (4*(npin % 8));
}

/*************** (C) COPYRIGHT ************** END OF FILE ********* D_EL *****/
