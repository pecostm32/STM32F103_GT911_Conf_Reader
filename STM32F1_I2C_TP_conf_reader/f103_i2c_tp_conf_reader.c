//Run openocd for debugging
//openocd -f ~/Data/NetbeansProjects/STM32/F103/STM32F1_I2C_TP_conf_reader/STM32F103C8T6.cfg

#include "stm32f103_db.h"
#include "f103_i2c_tp_conf_reader.h"
#include "usb.h"

#define STACK_TOP 0x20005000

extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

int main(void);
void resetHandler(void);

//Default pins for I2C2
#define SCL_PIN          10
#define SDA_PIN          11


const void * intVectors[76] __attribute__((section(".vectors"))) =
{
    (void*) STACK_TOP,
    resetHandler,
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    usbIrqHandler,
    0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0
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



#define I2C_BUFFER_SIZE      128


volatile uint8_t i2c_rx[I2C_BUFFER_SIZE]; //Buffer for i2c receive data
volatile uint8_t i2c_rx_in_idx = 0;       //Index for putting data into the i2c receive buffer
volatile uint8_t i2c_rx_out_idx = 0;      //Index for taking data from the i2c receive buffer. Set volatile since it changes in interrupt routine.

//Main processing loop
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
  
  //Enable the clocks for the used peripherals.
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
  
  //Need I2C interface turned on
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  
  //Enable the system timer for delay function
  //In default setting with 72MHz AHB clock this timer runs on 9MHz, so 111ns per tick. Takes 9 ticks for a microsecond
  STK->LOAD = 0x00FFFFFF;
  STK->CTRL = STK_CTRL_ENABLE;
  
  //Init the USB device
  usbInit();

  //IO pins need to be configured first
  //Pin with LED to show activity
  InitIOPin(GPIOC, 13, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);

  //Input to monitor RST signal
  InitIOPin(GPIOA, 0, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
  
  //Input to monitor INT signal
  InitIOPin(GPIOA, 1, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
  
  //Input to monitor SCL signal
  InitIOPin(GPIOB, SCL_PIN, GPIO_MODE_2MHZ, GPIO_CONF_AF_OD);
  
  //Input to monitor SDA signal
  InitIOPin(GPIOB, SDA_PIN, GPIO_MODE_2MHZ, GPIO_CONF_AF_OD);

  //Setup I2C interface
  I2C2->CR2 = I2C_CR2_FREQ_32MHZ;
  I2C2->CCR = I2C_CCR_32MHZ_100KHZ;
  I2C2->TRISE = 33;                   //Slow mode (100KHz) rise time max 1uS on 32MHz clock leads to 1uS / 31,25nS = 32 + 1 = 33
    
  I2C2->CR1 = I2C_CR1_PE;
  
  
  //Set priority for external 10:15 interrupt to be higher then the other interrupts
  //This is an array of 8 bit registers, of which only the upper 4 bits are used for the priority allowing for 16 levels
  //By grouping this is separated to allow for having sub priorities within a single group.
  //In the usb init this is set for 4 group priorities with each 4 sub priorities.
  //The higher the number the lower the priority
  //NVIC->IP[EXTI_15_10_IRQn] = 0x40;  //(0100b) Group priority 1, sub priority 0

  //Enable the external 15:10 interrupt
  //This is an array of 32 bit registers, only used to enable an interrupt. To disable the ICER registers need to be used
  //Each register serves 32 interrupts, so to get the register for the interrupt, shift the IRQ number right 5 times (divide by 32) and to get
  //the right interrupt enable bit, shift a unsigned 32 bit integer 1 the IRQ number anded with 31 (modulo 32) times to the right
  //NVIC->ISER[EXTI_15_10_IRQn >> 0x05] = (uint32_t)0x01 << (EXTI_15_10_IRQn & 0x1F);

  //Main loop to handle usb in and output  
  while(1)
  {
    int c = -1;
    int addr = 0x14;
    
    usbSend('P');
    usbSend('r');
    usbSend('e');
    usbSend('s');
    usbSend('s');
    usbSend(' ');
    usbSend('k');
    usbSend('e');
    usbSend('y');
    usbSend('\n');
    
    while(c == -1)
    {
      c = usbRead();
    }
    
    c = -1;
    
    //For reading the TP conf the following is needed
    //Start
    //Output 0x28 (write to slave 0x14)
    //Output 0x80
    //Output 0x47
    //Re-start
    //Output 0x29 (read from slave 0x14)
    //Read 185 bytes
    
    //Address slave for writing the register address
    if(sendi2cslaveaddres(addr, 0))
    {
      readi2cconfig(addr);
    }
    else
    {
      addr = 0x5D;
      
      if(sendi2cslaveaddres(addr, 0))
      {
        readi2cconfig(addr);
      }
      else
      {
        //Let the user know an error occurred
        usbSend('E');
        usbSend('1');
        usbSend('\n');
      }
    }
      
    //Write stop bit in CR1 to stop communication
    I2C2->CR1 |= I2C_CR1_STOP;
      
    //Wait 100uS before next address check
    usdelay(100);
  }
}

//mode: 0 = write, 1 = read
int sendi2cslaveaddres(uint8_t address, uint8_t mode)
{
  int retval = 0;
  int temp;
  
  //Write start bit in CR1 to start communication
  I2C2->CR1 |= I2C_CR1_START;

  //Wait until start has been send
  while((I2C2->SR1 & I2C_SR1_SB) != I2C_SR1_SB);

  //Write slave address to DR register
  I2C2->DR = ((address & 0x7F) << 1) | (mode & 0x01);

  //Wait until address has been send and acked or nack has been detected
  while(((I2C2->SR1 & I2C_SR1_ADDR) != I2C_SR1_ADDR) && ((I2C2->SR1 & I2C_SR1_AF) != I2C_SR1_AF));
  
  //Check if a device is connected to this address
  if(I2C2->SR1 & I2C_SR1_ADDR)
    retval = 1;
  
  //Clear nack flag
  I2C2->SR1 = 0;

  //Clear addr flag
  temp = I2C2->SR1;
  temp = I2C2->SR2;
  
  //Return the result
  return(retval);
}


int sendi2cbyte(uint8_t byte)
{
  //Write byte to DR register
  I2C2->DR = byte;

  //Wait until it has been send and acked or nack has been detected
  while(((I2C2->SR1 & I2C_SR1_TXE) != I2C_SR1_TXE) && ((I2C2->SR1 & I2C_SR1_AF) != I2C_SR1_AF));
  
  //Check if a acknowledged
  if(I2C2->SR1 & I2C_SR1_TXE)
    return(1);
  
  //Not acknowledged
  return(0);
}

//ack: 1 is acknowledge received byte, 0 = do not acknowledge byte
int readi2cbyte(uint8_t ack)
{
  //Setup for ack or nack
  I2C2->CR1 |= ((ack & 0x01) << 10);
  
  //Wait until byte has been received
  while((I2C2->SR1 & I2C_SR1_RXNE) != I2C_SR1_RXNE);
  
  return(I2C2->DR);
}

void readi2cconfig(uint8_t addr)
{
  int i;
  int byte;
  
  //Send first address byte
  if(sendi2cbyte(0x80))
  {
    //Send second address byte
    if(sendi2cbyte(0x47))
    {
      //Start the reading process
      if(sendi2cslaveaddres(addr, 1))
      {
        //Read all the config bytes
        for(i=0;i<185;i++)
        {
          byte = readi2cbyte(1);

          //Show received byte to the user
          usbSend('0');
          usbSend('x');
          printhexnibble((byte >> 4) & 0x0F);
          printhexnibble(byte & 0x0F);

          //Terminate the line
          usbSend('\n');
        }

        byte = readi2cbyte(0);

        //Show received byte to the user
        usbSend('0');
        usbSend('x');
        printhexnibble((byte >> 4) & 0x0F);
        printhexnibble(byte & 0x0F);

        //Terminate the line
        usbSend('\n');
      }
      else
      {
        //Let the user know an error occurred
        usbSend('E');
        usbSend('4');
        usbSend('\n');
      }
    }
    else
    {
      //Let the user know an error occurred
      usbSend('E');
      usbSend('3');
      usbSend('\n');
    }
  }
  else
  {
    //Let the user know an error occurred
    usbSend('E');
    usbSend('2');
    usbSend('\n');
  }
}

void printhexnibble(unsigned char nibble)
{
  //Check if needs to be converted to A-F character
  if(nibble > 9)
  {
    //To make alpha add 55. (55 = 'A' - 10)
    nibble += 55;
  }
  else
  {
    //To make digit add '0'
    nibble += '0';
  }

  usbSend(nibble);
}
