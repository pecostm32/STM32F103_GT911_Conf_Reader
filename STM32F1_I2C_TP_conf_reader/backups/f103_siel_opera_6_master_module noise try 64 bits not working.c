//Run openocd for debugging
//openocd -f ~/NetBeansProjects/STM_Drum_Box/STM32F103C8T6.cfg

#include "stm32f103_db.h"
#include "usb.h"
#include "sintab.h"
#include "../Includes/parameters.h"
#include "commands.h"

#define STACK_TOP 0x20005000

extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

int main(void);
void resetHandler(void);
//void tim1IrqHandler(void);
//void tim2IrqHandler(void);
//void tim3IrqHandler(void);

void dmach3IrqHandler(void);
void dmach5IrqHandler(void);

void SendString(char *str);
void SendDynamicsRouting();

const void * intVectors[76] __attribute__((section(".vectors"))) =
{
    (void*) STACK_TOP,
    resetHandler,
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,
    dmach3IrqHandler,
    0,
    dmach5IrqHandler,
    0,0,0,0,
    usbIrqHandler,
    0,0,0,0,
    0, //tim1IrqHandler,
    0,0,
    0, //tim2IrqHandler,
    0, //tim3IrqHandler,
    0,0,0,0,0,0,0,0,0
};

void resetHandler(void)
{
  unsigned char volatile *src;
  unsigned char volatile *dst;
  unsigned len;

  src= &INIT_DATA_VALUES;
  dst= &INIT_DATA_START;
  len= &INIT_DATA_END - &INIT_DATA_START;

  while(len--)
    *dst++ = *src++;

  dst = &BSS_START;

  len = &BSS_END - &BSS_START;

  while(len--)
    *dst++=0;

  main();
}

//The number of microseconds must be less then 1864135. (just over 1.8 second)
//Multiplied by 9 gives the number of ticks.
//With this it can only lead to a single timer overflow, which this function can handle
void usdelay(int32_t usec)
{
  int32_t end = STK->VAL - (usec * 9);

  //Check if there is the need to wait for an timer overflow
  if(end <= 0)
  {
    //Wait for the overflow to occur
    while((STK->CTRL & STK_CTRL_OVERFLOW) == 0);

    //calculate the new end value
    end += 0x00FFFFFF;
  }

  //Wait till the timer reaches the intended value for the given delay
  while(STK->VAL >= end);
}

//Simple function for setup of an IO pin
void InitIOPin(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t conf)
{
  //Mix the mode and configuration for single instruction usage
  uint32_t data = mode | (conf << 2);

  //Create a base pointer for either the lower or the higher control register
  __IO uint32_t *reg;

  //See if the lower control register or the higher control register needs to be used
  if(pin < 8)
  {
    //Low control register used for first 8 pins
    reg = &port->CRL;
  }
  else
  {
    //Force pin into 8 pins per register range
    pin -= 8;

    //High control register used for upper 8 pins
    reg = &port->CRH;
  }

  //4 control bits used per pin
  pin *= 4;

  //Reset bits first and set new mode and configuration.
  *reg &= ~(0x0F << pin);
  *reg |=  (data << pin);
}


//SPI1 is used to stream the high frequency signals. For now 2 signals are routed via this channel
//With 36MHz ABP1 clock and divider set to 8 (BR = 2) the word rate on SPI1 is going to be 281250 wps (words per second)
//This means a signal sample rate of 140625Hz (samples per second)
//On the DMA halfway and finished interrupt new signal values need to be calculated. This way a double buffered setup is achieved

//SPI2 is used to stream the low frequency signals. For now 5 LFO outputs are routed via this channel
//With 36MHz ABP1 clock and divider set to 32 (BR = 4) the word rate on SPI2 is going to be 70312,5 wps (words per second)
//This means a signal sample rate of 14062,5Hz (samples per second)
//On the DMA halfway and finished interrupt new signal values need to be calculated. This way a double buffered setup is achieved


//Footage can also be included in these signals. They give a shift of 2 octaves
//For LFO I and II the signals are made from four components. Three semi static parameters and the LFO sine output
//The pitch bend wheel (not the midi pitch bend based on the schematics) is routed to the DCO's based on a select switch.
//The depth wheel can amplify the effect of the lfo output.
//The master tune is always a part of the signal
//So the signal is, depending on the settings, the LFO output multiplied by the sum of the two depth settings plus the pitch bend value plus the master tune
//The two depth settings have equal influence (0 to 1, so max depth is 2)

//Pitch bend has a range of 3 semi-tones up or down
//Master tune has a range of 1 semi-tone up or down
//LFO has a range of max 2 semi-tones up or down

//For LFO II signal there is also the detune of DCO B to take into account
//Course has a range of 8 semi-tones down
//Fine has a range of half a semi-tone up or down

//For footage the multiplication factor is one of: 1, 2, 4
//Standard this signal should be 1. When there is a octave shift up (16' is lowest, then 8' and 4' is highest) it becomes 2 or 4 
//This means a need for a total of 14,5 semi-tones down and 30,5 semi-tones up (two octaves plus 6.5 semi-tones)
//Up is * 5,822612671
//Down is * 0,432768283

//For pitch this means three bits need to be used for the decimal. The rest is for the mantissa.

//With this in mind and an output frequency range for the oscillators of min 14,151522844Hz to max 6092,781775109

//The key number should provide a multiplication factor based on the semi-tone scale. This factor needs to be multiplied with the pitch factor
//The result represents the needed frequency.

//HFO calculation needs to be done with 32 bits. 23 for the mantissa part and 9 for the fraction
//Multiplication is then a bit hard to do

//HFO max is 7963266,006916504Hz
//HFO min is 2367493,165381729Hz

//For now 14 bits for each signal gives a range of 16 bits in total
//So 14 bits sine table is needed.


//For the generation of the voice signals a sampling rate is needed. Hardware as it is now provides a voice clock but this is not
//usable in a sample based system. Can't make the waves with the use of a timer as first conceived.
//So will use the SPI clock to derive the sample clock from. The high frequency signals are streamed on the SPI 1. Two signals are transmitted.
//One is for the noise signal, which needs to be passed through all the devices. The second one is for the audio signal. This is an addition
//of all the separate voices. Basically an add and divide by two is needed in every module. The divide can be omitted by scaling the signals such
//that the total sum will be below the maximum allowed in the output module. (Top 12 of 16 bits)


//Buffer for two times the five low frequency signals. (DCO A tuning, DCO B tuning, DCO A PWM, DCO B PWM, VCF cutoff)
volatile int16_t lowfrequencydata[NOF_LF_WORDS] = { 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00000, 0x0000, 0x0000 };

//Buffer for two times two high frequency signals. (Voice, Noise)
volatile int16_t highfrequencydata[NOF_HF_WORDS] = { 0x0000, 0x0000, 0x0000, 0x0000 };

//For random number sequencing
volatile uint32_t randx=132456789;
volatile uint32_t randy=362436069;
volatile uint32_t randz=521288629;

//Parameter data storage
volatile tagMasterData masterparameters;
volatile tagVoiceData  voiceparameters;

volatile uint16_t curchar;

int main(void)
{
  //Setup flash to work with 72MHz clock
  //Enable the Prefetch Buffer and Set to 2 wait states
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  //Configure system clock
  //External oscillator: 8MHz
  //PLL multiplicator: x9
  //SYSCLK: 64MHz
  //AHB: SYSCLK = 72MHz
  //APB1: SYSCLK/2 = 36MHz  //Timer 2,3 and 4 run on 72MHz since APB1 divider is not 1
  //APB2: SYSCLK/2 = 36MHz  //Timer 1 also runs on 72MHz since APB2 divider is not 1
  //ADC: SYSCLK/6 = 12MHz
  //USB: SYSCLK/1.5 = 48MHz
  RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;

  //Enable external oscillator
  RCC->CR |= RCC_CR_HSEON;

  //Wait for the clock to become stable
  while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

  //Enable the PLL
  RCC->CR |= RCC_CR_PLLON;

  //Wait for the PLL to become stable
  while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

  //Switch to the PLL clock as system clock source. Since on reset these bits are set to 0 no need to clear first.
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  //Wait for the PLL to become the clock source
  while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

  //From this point on it is not possible to change the clock configuration without switching back to HSI
  
  //Enable the clocks for the used peripherals. PORTA for USART1, PORTB for USART3, PORTC for the led, USART1, SPI1, Alternate Function IO and TIM1
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN;

  //Enable USART2, TIM2, TIM3 and SPI2
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_SPI2EN;

  //Enable the DMA
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  
  //Enable the system timer for delay function
  //In default setting with 72MHz AHB clock this timer runs on 9MHz, so 111ns per tick. Takes 9 ticks for a microsecond
  STK->LOAD = 0x00FFFFFF;
  STK->CTRL = STK_CTRL_ENABLE;
  
  //Init the USB device
  usbInit();

  //IO pins need to be configured first
  //In reset mode IO is floating. After configuration as input with pull up or down it will take on reset setting which is pull down
  //In this case it is needed to be pull up so we set it before pin configuration
  //Enable pull up on PA0
  GPIOA->ODR |= 0x0001;
  
  //Measurement pin for interrupt timing measurement
  InitIOPin(GPIOB, 5, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  InitIOPin(GPIOB, 6, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  
  //Pin with LED to show activity
  InitIOPin(GPIOC, 13, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);

  //General purpose input with pull up / down
  InitIOPin(GPIOA, 0, GPIO_MODE_INPUT, GPIO_CONF_I_PUPD);  //Voice module ready input
  
  //USART 1 TX
  InitIOPin(GPIOA, 9, GPIO_MODE_10MHZ, GPIO_CONF_AF_PP);   //Midi out
  
  //USART 1 RX
  InitIOPin(GPIOA, 10, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);  //Midi in

  //SP1 SCK
  InitIOPin(GPIOA, 5, GPIO_MODE_50MHZ, GPIO_CONF_AF_PP);   //Audio stream clock output

  //SP1 MOSI
  InitIOPin(GPIOA, 7, GPIO_MODE_50MHZ, GPIO_CONF_AF_PP);   //Audio stream data output

  //USART 2 TX
  InitIOPin(GPIOA, 2, GPIO_MODE_10MHZ, GPIO_CONF_AF_PP);   //Configuration and parameter data output
  
  //SPI 2 SCK
  InitIOPin(GPIOB, 13, GPIO_MODE_50MHZ, GPIO_CONF_AF_PP);  //Low speed signal stream clock output
  
  //SPI 2 MOSI
  InitIOPin(GPIOB, 15, GPIO_MODE_50MHZ, GPIO_CONF_AF_PP);  //Low speed signal stream data output
  
  /*
  //Timer 1 is used to make the carrier for the sine instruments. It gives a frequency of 70312,5Hz
  TIM1->PSC = 0;
  TIM1->CNT = 0;
  TIM1->ARR = 1023;
  TIM1->CCR1 = 512; //Duty cycle control
  TIM1->RCR = 4;    //Interrupt generated only every 5th reload

  //Timer 1 up counting with compare output on channel 1. Output in PWM1 mode and preload enabled
  TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  TIM1->CCER = TIM_CCER_CC1E;
  TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM1->DIER = TIM_DIER_UIE;
  TIM1->BDTR = TIM_BDTR_MOE;  //Enable outputs

  //Timer 2
  TIM2->CNT = 0;
  TIM2->PSC = 15999;  //72MHz / 16000 = 4500Hz
  TIM2->ARR = 1124;   //4500Hz / 1125 = 4Hz;
  TIM2->CCR1 = 562;   //50% duty cycle on the output;

  //Timer 2 generates an interrupt at a 2Hz rate
  TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  TIM2->CCER = TIM_CCER_CC1E;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

  //Timer 3 is used to make the carrier for noise instruments. It gives a frequency of 70312,5Hz
  TIM3->PSC = 0;
  TIM3->CNT = 383;
  TIM3->ARR = 1023;
  TIM3->CCR1 = 512; //Duty cycle control
  TIM3->RCR = 1;    //Interrupt generated only every 2th reload

  //Timer 3 up counting with compare output on channel 1. Output in PWM1 mode and preload enabled
  TIM3->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  TIM3->CCER = TIM_CCER_CC1E;
  TIM3->DIER = TIM_DIER_UIE;
  TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
*/

  //Setup the DMA for the high frequency signal stream
  //Set the address of the slave SPI data register as DMA destination and the high frequency data as source
  DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel3->CMAR = (uint32_t)highfrequencydata;
  DMA1_Channel3->CNDTR = NOF_HF_WORDS;
  //Memory increment with circular buffer transferring from memory 16 bits per transfer
  DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_MSIZE_0 |  DMA_CCR_PSIZE_0 | DMA_CCR_HTIE | DMA_CCR_TCIE; // | DMA_CCR_EN;
  
  //Enable the low frequency signal stream interface and setup for master mode, clock/8, falling edge low start clock, 16 bit, msb first out and transmit only
  SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_BR_DIV8 | SPI_CR1_CPOL | SPI_CR1_DFF | SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN;
  
  //Setup the DMA for the low frequency signal stream
  //Set the address of the slave SPI data register as DMA destination and the low frequency data as source
  DMA1_Channel5->CPAR = (uint32_t)&SPI2->DR;
  DMA1_Channel5->CMAR = (uint32_t)lowfrequencydata;
  DMA1_Channel5->CNDTR = NOF_LF_WORDS;
  //Memory increment with circular buffer transferring from memory 16 bits per transfer
  DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_MSIZE_0 |  DMA_CCR_PSIZE_0 | DMA_CCR_HTIE | DMA_CCR_TCIE; // | DMA_CCR_EN;
  
  //Enable the low frequency signal stream interface and setup for master mode, clock/32, falling edge low start clock, 16 bit, msb first out and transmit only
  SPI2->CR1 = SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_BR_DIV32 | SPI_CR1_CPOL | SPI_CR1_DFF | SPI_CR1_SSM | SPI_CR1_SSI;
  SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN;

  //Setup DMA for USART2 transmission
  //Set the address of the USART data register as DMA destination and the parameter data as source
  DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;
  DMA1_Channel7->CMAR = (uint32_t)&voiceparameters;
  DMA1_Channel7->CNDTR = sizeof(voiceparameters);
  //Memory increment with circular buffer transferring from memory 8 bits per transfer
  DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR; // | DMA_CCR_EN;
  
  //Enable the parameter stream interface. Setup for asynchronous transmission of 8 bits, 1 stop bit, no parity. 2250000 bits per second
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
  USART2->CR3 = USART_CR3_DMAT;
  USART2->BRR = 16;

  //Set priority for timer 1, 2 and 3 interrupt to be higher then the other interrupts
  //This is an array of 8 bit registers, of which only the upper 4 bits are used for the priority allowing for 16 levels
  //By grouping this is separated to allow for having sub priorities within a single group.
  //In the usb init this is set for 4 group priorities with each 4 sub priorities.
  //The higher the number the lower the priority
//  NVIC->IP[TIM1_UP_IRQn] = 0x80;  //(1000b) Group priority 2, sub priority 0
//  NVIC->IP[TIM2_IRQn]    = 0x90;  //(1001b) Group priority 2, sub priority 1
//  NVIC->IP[TIM3_IRQn]    = 0x40;  //(0100b) Group priority 1, sub priority 0
  NVIC->IP[DMA1_Channel3_IRQn] = 0x50;  //(0101b) Group priority 1, sub priority 1
  NVIC->IP[DMA1_Channel5_IRQn] = 0x40;  //(0100b) Group priority 1, sub priority 0
  
  //Enable the timer 1 and 2 interrupt
  //This is an array of 32 bit registers, only used to enable an interrupt. To disable the ICER registers need to be used
  //Each register serves 32 interrupts, so to get the register for the interrupt, shift the IRQ number right 5 times (divide by 32) and to get
  //the right interrupt enable bit, shift a unsigned 32 bit integer 1 the IRQ number anded with 31 (modulo 32) times to the right
//  NVIC->ISER[TIM1_UP_IRQn >> 0x05] = (uint32_t)0x01 << (TIM1_UP_IRQn & 0x1F);
//  NVIC->ISER[TIM2_IRQn >> 0x05] = (uint32_t)0x01 << (TIM2_IRQn & 0x1F);
//  NVIC->ISER[TIM3_IRQn >> 0x05] = (uint32_t)0x01 << (TIM3_IRQn & 0x1F);
  NVIC->ISER[DMA1_Channel3_IRQn >> 0x05] = (uint32_t)0x01 << (DMA1_Channel3_IRQn & 0x1F);
  NVIC->ISER[DMA1_Channel5_IRQn >> 0x05] = (uint32_t)0x01 << (DMA1_Channel5_IRQn & 0x1F);

  //For synchronization with the slave modules in is necessary to wait for them
  //Wait for PA0 to go low
  while(GPIOA->IDR & 0x0001);
  
  //Wait for PA0 to go high again
  while((GPIOA->IDR & 0x0001) == 0);
  
  //Voice modules are ready so start streaming
  DMA1_Channel3->CCR |= DMA_CCR_EN;
  DMA1_Channel5->CCR |= DMA_CCR_EN;
  DMA1_Channel7->CCR |= DMA_CCR_EN;
  
  while(1)
  {
    //Read a character from the USB receive buffer
    curchar = usbRead();
    
    //Check if any data to process
    if(curchar != -1)
    {
      handleinput();
    }    
  }
}

//On DMA channel 3 halfway and finished interrupt new high frequency signal samples are calculated
void dmach3IrqHandler(void)
{
  int32_t noiselevel = (masterparameters.NoiseLevel << 16) + 66;  //A minimum noise level of approximate 0.001 is added (66/65536)
  int16_t  noise = 0;
  uint32_t randt;

  //Set pin for time measurement
  GPIOB->ODR |= 1 << 5;
  
  //Generate random number for noise
  randx ^= randx << 16;
  randx ^= randx >> 5;
  randx ^= randx << 1;

  randt = randx;
  randx = randy;
  randy = randz;
  randz = randt ^ randx ^ randy;

  //Noise max signal on the bus is signed 16 bits
  //The level control has 256 steps
  //Minimum noise for filter self oscillation is 0.001 times the full noise signal
  //So for calculations minimum multiplier is 0.001 and max is 255.001
  //This means noiselevel + 0.001
  //In fixed point this means at least a 10 bit mantissa + 8 bit integer times the random number should be shifted back 18 places
  //Here a 16 bit mantissa + 8 bit integer is used so shift with 24
  noise = (int16_t)((int64_t)(randz * noiselevel) >> 24);
  //noise = noiselevel >> 8;
  
  //Check on DMA flags to see if half way and if not check on fully done
  if(DMA1->ISR & DMA_ISR_HTIF3)
  {
    //Modify first half of the buffer
    highfrequencydata[1] = noise;
  }
  else if(DMA1->ISR & DMA_ISR_TCIF3)
  {
    //Modify second half of the buffer
    highfrequencydata[3] = noise;
  }

  //Clear all the possible channel5 interrupts
  DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;
  
  //Clear pin for time measurement
  GPIOB->ODR &= ~(1 << 5);
}

//On DMA channel 5 halfway and finished interrupt new low frequency signal samples are calculated
void dmach5IrqHandler(void)
{
  //Set pin for time measurement
  GPIOB->ODR |= 1 << 6;
    
  //Check on DMA flags to see if half way and else check on fully done
  if(DMA1->ISR & DMA_ISR_HTIF5)
  {
    //Modify first half of the buffer
//    lowfrequencydata[0] = 0x0;
  }
  else if(DMA1->ISR & DMA_ISR_TCIF5)
  {
    //Modify second half of the buffer
//    lowfrequencydata[3] = 0x0;
  }

  //Clear all the possible channel5 interrupts
  DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;
  
  //Clear pin for time measurement
  GPIOB->ODR &= ~(1 << 6);
}
