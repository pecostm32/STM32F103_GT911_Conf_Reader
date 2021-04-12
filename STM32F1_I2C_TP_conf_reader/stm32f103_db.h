#ifndef __STM32F103_H_
#define __STM32F103_H_

#include <stdint.h>

#define     __IO    volatile

#define     __IM     volatile const      //Defines 'read only' structure member permissions
#define     __OM     volatile            //Defines 'write only' structure member permissions
#define     __IOM    volatile            //Defines 'read / write' structure member permissions

//Power Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;


//Reset and Clock Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

//FLASH Registers
typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

//Real-Time Clock
typedef struct
{
  __IO uint32_t CRH;
  __IO uint32_t CRL;
  __IO uint32_t PRLH;
  __IO uint32_t PRLL;
  __IO uint32_t DIVH;
  __IO uint32_t DIVL;
  __IO uint32_t CNTH;
  __IO uint32_t CNTL;
  __IO uint32_t ALRH;
  __IO uint32_t ALRL;
} RTC_TypeDef;

//General Purpose I/O
typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

//TIM Timers
typedef struct
{
  __IO uint32_t CR1;             //TIM control register 1,                      Address offset: 0x00
  __IO uint32_t CR2;             //TIM control register 2,                      Address offset: 0x04
  __IO uint32_t SMCR;            //TIM slave Mode Control register,             Address offset: 0x08
  __IO uint32_t DIER;            //TIM DMA/interrupt enable register,           Address offset: 0x0C
  __IO uint32_t SR;              //TIM status register,                         Address offset: 0x10
  __IO uint32_t EGR;             //TIM event generation register,               Address offset: 0x14
  __IO uint32_t CCMR1;           //TIM  capture/compare mode register 1,        Address offset: 0x18
  __IO uint32_t CCMR2;           //TIM  capture/compare mode register 2,        Address offset: 0x1C
  __IO uint32_t CCER;            //TIM capture/compare enable register,         Address offset: 0x20
  __IO uint32_t CNT;             //TIM counter register,                        Address offset: 0x24
  __IO uint32_t PSC;             //TIM prescaler register,                      Address offset: 0x28
  __IO uint32_t ARR;             //TIM auto-reload register,                    Address offset: 0x2C
  __IO uint32_t RCR;             //TIM  repetition counter register,            Address offset: 0x30
  __IO uint32_t CCR1;            //TIM capture/compare register 1,              Address offset: 0x34
  __IO uint32_t CCR2;            //TIM capture/compare register 2,              Address offset: 0x38
  __IO uint32_t CCR3;            //TIM capture/compare register 3,              Address offset: 0x3C
  __IO uint32_t CCR4;            //TIM capture/compare register 4,              Address offset: 0x40
  __IO uint32_t BDTR;            //TIM break and dead-time register,            Address offset: 0x44
  __IO uint32_t DCR;             //TIM DMA control register,                    Address offset: 0x48
  __IO uint32_t DMAR;            //TIM DMA address for full transfer register,  Address offset: 0x4C
  __IO uint32_t OR;              //TIM option register,                         Address offset: 0x50
}TIM_TypeDef;

//SPI
typedef struct
{
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t CRCPR;
  __IO uint32_t RXCRCR;
  __IO uint32_t TXCRCR;
  __IO uint32_t I2SCFGR;
} SPI_TypeDef;

//USART
typedef struct
{
  __IO uint32_t SR;         //USART Status register
  __IO uint32_t DR;         //USART Data register
  __IO uint32_t BRR;        //USART Baud rate register
  __IO uint32_t CR1;        //USART Control register 1
  __IO uint32_t CR2;        //USART Control register 2
  __IO uint32_t CR3;        //USART Control register 3
  __IO uint32_t GTPR;       //USART Guard time and prescaler register
} USART_TypeDef;

//DMA channel
typedef struct
{
  __IO uint32_t CCR;          //DMA channel x configuration register            
  __IO uint32_t CNDTR;        //DMA channel x number of data register           
  __IO uint32_t CPAR;         //DMA channel x peripheral address register       
  __IO uint32_t CMAR;         //DMA channel x memory address register           
} DMA_Channel_TypeDef;

//DMA
typedef struct
{
  __IO uint32_t ISR;          //DMA interrupt status register,                            Address offset: 0x00
  __IO uint32_t IFCR;         //DMA interrupt flag clear register,                        Address offset: 0x04
} DMA_TypeDef;

//External Interrupt/Event Controller
typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;

//Structure type to access the System Control Block (SCB).
typedef struct
{
  __IM  uint32_t CPUID;                  //Offset: 0x000 (R/ )  CPUID Base Register
  __IOM uint32_t ICSR;                   //Offset: 0x004 (R/W)  Interrupt Control and State Register
  __IOM uint32_t VTOR;                   //Offset: 0x008 (R/W)  Vector Table Offset Register
  __IOM uint32_t AIRCR;                  //Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
  __IOM uint32_t SCR;                    //Offset: 0x010 (R/W)  System Control Register
  __IOM uint32_t CCR;                    //Offset: 0x014 (R/W)  Configuration Control Register
  __IOM uint8_t  SHP[12U];               //Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15)
  __IOM uint32_t SHCSR;                  //Offset: 0x024 (R/W)  System Handler Control and State Register
  __IOM uint32_t CFSR;                   //Offset: 0x028 (R/W)  Configurable Fault Status Register
  __IOM uint32_t HFSR;                   //Offset: 0x02C (R/W)  HardFault Status Register
  __IOM uint32_t DFSR;                   //Offset: 0x030 (R/W)  Debug Fault Status Register
  __IOM uint32_t MMFAR;                  //Offset: 0x034 (R/W)  MemManage Fault Address Register
  __IOM uint32_t BFAR;                   //Offset: 0x038 (R/W)  BusFault Address Register
  __IOM uint32_t AFSR;                   //Offset: 0x03C (R/W)  Auxiliary Fault Status Register
  __IM  uint32_t PFR[2U];                //Offset: 0x040 (R/ )  Processor Feature Register
  __IM  uint32_t DFR;                    //Offset: 0x048 (R/ )  Debug Feature Register
  __IM  uint32_t ADR;                    //Offset: 0x04C (R/ )  Auxiliary Feature Register
  __IM  uint32_t MMFR[4U];               //Offset: 0x050 (R/ )  Memory Model Feature Register
  __IM  uint32_t ISAR[5U];               //Offset: 0x060 (R/ )  Instruction Set Attributes Register
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  //Offset: 0x088 (R/W)  Coprocessor Access Control Register
} SCB_Type;

//Structure type to access the Nested Vectored Interrupt Controller (NVIC)
typedef struct
{
  __IOM uint32_t ISER[8U];               //Offset: 0x000 (R/W)  Interrupt Set Enable Register
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               //Offset: 0x080 (R/W)  Interrupt Clear Enable Register
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               //Offset: 0x100 (R/W)  Interrupt Set Pending Register
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               //Offset: 0x180 (R/W)  Interrupt Clear Pending Register
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               //Offset: 0x200 (R/W)  Interrupt Active bit Register
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               //Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide)
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   //Offset: 0xE00 ( /W)  Software Trigger Interrupt Register
}  NVIC_Type;

//Structure to access Alternate Function I/O
typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
  uint32_t RESERVED0;
  __IO uint32_t MAPR2;  
} AFIO_TypeDef;

//Structure to access I2C registers
typedef struct
{
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t OAR1;
  __IO uint32_t OAR2;
  __IO uint32_t DR;
  __IO uint32_t SR1;
  __IO uint32_t SR2;
  __IO uint32_t CCR;
  __IO uint32_t TRISE;
} I2C_TypeDef;

//Structure type to access the System Timer (Systick)
typedef struct
{
  __IOM uint32_t CTRL;               //Offset: 0x000 (R/W)  System timer control
  __IOM uint32_t LOAD;               //Offset: 0x004 (R/W)  System timer reload
  __IOM uint32_t VAL;                //Offset: 0x008 (R/W)  System timer value
  __IOM uint32_t CALIB;              //Offset: 0x000 (R/W)  System timer calibration
}  STK_Type;

//Universal Serial Bus Full Speed Device
typedef struct
{
  __IO uint16_t EP0R;                 //USB Endpoint 0 register,         Address offset: 0x00
  __IO uint16_t RESERVED0;            //Reserved
  __IO uint16_t EP1R;                 //USB Endpoint 1 register,         Address offset: 0x04
  __IO uint16_t RESERVED1;            //Reserved
  __IO uint16_t EP2R;                 //USB Endpoint 2 register,         Address offset: 0x08
  __IO uint16_t RESERVED2;            //Reserved
  __IO uint16_t EP3R;                 //USB Endpoint 3 register,         Address offset: 0x0C
  __IO uint16_t RESERVED3;            //Reserved
  __IO uint16_t EP4R;                 //USB Endpoint 4 register,         Address offset: 0x10
  __IO uint16_t RESERVED4;            //Reserved
  __IO uint16_t EP5R;                 //USB Endpoint 5 register,         Address offset: 0x14
  __IO uint16_t RESERVED5;            //Reserved
  __IO uint16_t EP6R;                 //USB Endpoint 6 register,         Address offset: 0x18
  __IO uint16_t RESERVED6;            //Reserved
  __IO uint16_t EP7R;                 //USB Endpoint 7 register,         Address offset: 0x1C
  __IO uint16_t RESERVED7[17];        //Reserved
  __IO uint16_t CNTR;                 //Control register,                Address offset: 0x40
  __IO uint16_t RESERVED8;            //Reserved
  __IO uint16_t ISTR;                 //Interrupt status register,       Address offset: 0x44
  __IO uint16_t RESERVED9;            //Reserved
  __IO uint16_t FNR;                  //Frame number register,           Address offset: 0x48
  __IO uint16_t RESERVEDA;            //Reserved
  __IO uint16_t DADDR;                //Device address register,         Address offset: 0x4C
  __IO uint16_t RESERVEDB;            //Reserved
  __IO uint16_t BTABLE;               //Buffer Table address register,   Address offset: 0x50
  __IO uint16_t RESERVEDC;            //Reserved
} USB_TypeDef;

//Packet memory entry
typedef struct
{
  __IO uint16_t DATA;
       uint16_t RESERVED;
}PMA_ENTRY;

//Structure for PMA
typedef struct
{
  PMA_ENTRY MEMORY[256];
} PMA_TypeDef;

//Structure for Buffer Descriptor Table
typedef struct
{
  PMA_ENTRY TX_ADDRESS;
  PMA_ENTRY TX_COUNT;
  PMA_ENTRY RX_ADDRESS;
  PMA_ENTRY RX_COUNT;
} BTABLE_ENTRY;

//Structure for endpoint descriptors
typedef struct
{
  BTABLE_ENTRY EPD[8];
} BTABLE_TypeDef;

typedef union
{
  uint16_t word;
  struct BYTES
  {
    uint8_t low;
    uint8_t high;
  } bytes;
} wbcombi;


#define PMA_BASE    (0x40006000L)  //USB_IP Packet Memory Area base address

#define PMA         ((PMA_TypeDef *) PMA_BASE)
#define EPBTABLE    ((BTABLE_TypeDef *) PMA_BASE)



#define RCC_BASE   0x40021000
#define PWR_BASE   0x40007000
#define RTC_BASE   0x40002800
#define EXTI_BASE  0x40010400

#define USB_BASE   0x40005C00

#define AFIO_BASE  0x40010000
#define GPIOA_BASE 0x40010800
#define GPIOB_BASE 0x40010C00
#define GPIOC_BASE 0x40011000

#define ADC1_BASE  0x40012400

#define TIM1_BASE  0x40012C00

#define TIM2_BASE  0x40000000
#define TIM3_BASE  0x40000400


#define SPI1_BASE  0x40013000
#define SPI2_BASE  0x40003800


#define DMA1_BASE             0x40020000
#define DMA1_Channel1_BASE    0x40020008
#define DMA1_Channel2_BASE    0x4002001C
#define DMA1_Channel3_BASE    0x40020030
#define DMA1_Channel4_BASE    0x40020044
#define DMA1_Channel5_BASE    0x40020058
#define DMA1_Channel6_BASE    0x4002006C
#define DMA1_Channel7_BASE    0x40020080


#define USART1_BASE           0x40013800
#define USART2_BASE           0x40004400
#define USART3_BASE           0x40004800

#define I2C1_BASE             0x40005400
#define I2C2_BASE             0x40005800

#define NVIC_BASE             0xE000E100
#define SCB_BASE              0xE000ED00                    //System Control Block Base Address
#define STK_BASE              0xE000E010                    //System Timer Block Base Address

#define FLASH_BASE            0x40022000

#define FLASH_START           0x08000000

#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)

#define TIM1                ((TIM_TypeDef *)TIM1_BASE)
#define TIM2                ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                ((TIM_TypeDef *)TIM3_BASE)

#define SPI1                ((SPI_TypeDef *)SPI1_BASE)
#define SPI2                ((SPI_TypeDef *)SPI2_BASE)

#define USART1              ((USART_TypeDef *)USART1_BASE)
#define USART2              ((USART_TypeDef *)USART2_BASE)
#define USART3              ((USART_TypeDef *)USART3_BASE)

#define I2C1                ((I2C_TypeDef *)I2C1_BASE)
#define I2C2                ((I2C_TypeDef *)I2C2_BASE)

#define DMA1                ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)


#define PWR                 ((PWR_TypeDef *)PWR_BASE)
#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define USB                 ((USB_TypeDef *)USB_BASE)
#define FLASH               ((FLASH_TypeDef *)FLASH_BASE)
#define RTC                 ((RTC_TypeDef *)RTC_BASE)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE)
#define SCB                 ((SCB_Type *)SCB_BASE)
#define NVIC                ((NVIC_Type *)NVIC_BASE)
#define STK                 ((STK_Type *)STK_BASE)

#define AFIO                ((AFIO_TypeDef *)AFIO_BASE)



#define DMA1_Channel3_IRQn    13
#define DMA1_Channel5_IRQn    15

#define USB_LP_CAN1_RX0_IRQn  20
#define TIM1_UP_IRQn          25
#define TIM2_IRQn             28
#define TIM3_IRQn             29

#define EXTI_15_10_IRQn       40

#define STK_CTRL_ENABLE       0x00000001               //Enable the system timer
#define STK_CTRL_INT_ENABLE   0x00000002               //Enable the system timer interrupt
#define STK_CTRL_CLK_CPU      0x00000004               //Select the cpu clock as system timer input clock
#define STK_CTRL_OVERFLOW     0x00010000               //Overflow flag

#define FLASH_ACR_LATENCY_2   0x00000004
#define FLASH_ACR_PRFTBE      0x00000010               //Prefetch Buffer Enable

#define RCC_CFGR_PLLMULL9     0x001C0000               //PLL input clock*9
#define RCC_CFGR_PLLSRC       0x00010000               //PLL entry clock source
#define RCC_CFGR_ADCPRE_DIV6  0x00008000               //PCLK2 divided by 6
#define RCC_CFGR_PPRE1_DIV2   0x00000400               //HCLK divided by 2
#define RCC_CFGR_PPRE2_DIV2   0x00002000               //HCLK divided by 2

#define RCC_CR_HSEON          0x00010000               //External High Speed clock enable
#define RCC_CR_HSERDY         0x00020000               //External High Speed clock ready flag

#define RCC_CR_PLLON          0x01000000               //PLL enable
#define RCC_CR_PLLRDY         0x02000000               //PLL clock ready flag

#define RCC_CFGR_SW_PLL       0x00000002               //PLL selected as system clock
#define RCC_CFGR_SWS_PLL      0x00000008               //PLL used as system clock

// Bit definition for RCC_APB2ENR register
#define RCC_APB2ENR_AFIOEN                   0x00000001              //Alternate Function I/O clock enable
#define RCC_APB2ENR_IOPAEN                   0x00000004              //I/O port A clock enable
#define RCC_APB2ENR_IOPBEN                   0x00000008              //I/O port B clock enable
#define RCC_APB2ENR_IOPCEN                   0x00000010              //I/O port C clock enable
#define RCC_APB2ENR_IOPDEN                   0x00000020              //I/O port D clock enable
#define RCC_APB2ENR_IOPEEN                   0x00000040              //I/O port E clock enable
#define RCC_APB2ENR_ADC1EN                   0x00000200              //ADC 1 interface clock enable
#define RCC_APB2ENR_ADC2EN                   0x00000400              //ADC 2 interface clock enable
#define RCC_APB2ENR_TIM1EN                   0x00000800              //TIM1 Timer clock enable
#define RCC_APB2ENR_SPI1EN                   0x00001000              //SPI 1 clock enable
#define RCC_APB2ENR_USART1EN                 0x00004000              //USART1 clock enable

//Bit definition for RCC_APB1ENR register
#define RCC_APB1ENR_TIM2EN                   0x00000001              //Timer 2 clock enabled
#define RCC_APB1ENR_TIM3EN                   0x00000002              //Timer 3 clock enable
#define RCC_APB1ENR_TIM4EN                   0x00000004              //Timer 4 clock enable
#define RCC_APB1ENR_WWDGEN                   0x00000800              //Window Watchdog clock enable
#define RCC_APB1ENR_USART2EN                 0x00020000              //USART 2 clock enable
#define RCC_APB1ENR_USART3EN                 0x00040000              //USART 3 clock enable
#define RCC_APB1ENR_I2C1EN                   0x00200000              //I2C 1 clock enable
#define RCC_APB1ENR_I2C2EN                   0x00400000              //I2C 2 clock enable
#define RCC_APB1ENR_CAN1EN                   0x02000000              //CAN1 clock enable
#define RCC_APB1ENR_SPI2EN                   0x00004000              //SPI 2 clock enable
#define RCC_APB1ENR_USBEN                    0x00800000              //USB Device clock enable
#define RCC_APB1ENR_BKPEN                    0x08000000              //Backup interface clock enable
#define RCC_APB1ENR_PWREN                    0x10000000              //Power interface clock enable


#define RCC_AHBENR_DMA1EN                    0x00000001              //DMA1 clock enable


#define PWR_CR_DBP                           0x00000100              //Disable Backup Domain write protection

//Bit definition for RCC_CSR register
#define RCC_CSR_LSION                        0x00000001              //Internal Low Speed oscillator enable
#define RCC_CSR_LSIRDY                       0x00000002              //Internal Low Speed oscillator Ready
#define RCC_CSR_RMVF                         0x01000000              //Remove reset flag
#define RCC_CSR_PINRSTF                      0x04000000              //PIN reset flag
#define RCC_CSR_PORRSTF                      0x08000000              //POR/PDR reset flag
#define RCC_CSR_SFTRSTF                      0x10000000              //Software Reset flag
#define RCC_CSR_IWDGRSTF                     0x20000000              //Independent Watchdog reset flag
#define RCC_CSR_WWDGRSTF                     0x40000000              //Window watchdog reset flag
#define RCC_CSR_LPWRRSTF                     0x80000000              //Low-Power reset flag

//Bit definition for RCC_BDCR register
#define RCC_BDCR_LSEON                       0x00000001              //External Low Speed oscillator enable
#define RCC_BDCR_LSERDY                      0x00000002              //External Low Speed oscillator Ready
#define RCC_BDCR_LSEBYP                      0x00000004              //External Low Speed oscillator Bypass

//RTC configuration
#define RCC_BDCR_RTCSEL                      0x00000300              //RTCSEL[1:0] bits (RTC clock source selection)
#define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000              //No clock
#define RCC_BDCR_RTCSEL_LSE                  0x00000100              //LSE oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_LSI                  0x00000200              //LSI oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_HSE                  0x00000300              //HSE oscillator clock divided by 128 used as RTC clock

#define RCC_BDCR_RTCEN                       0x00008000              //RTC clock enable
#define RCC_BDCR_BDRST                       0x00010000              //Backup domain software reset

#define RTC_CRL_RSF                          0x00000008               //Registers Synchronized Flag
#define RTC_CRL_CNF                          0x00000010               //Configuration Flag
#define RTC_CRL_RTOFF                        0x00000020               //RTC operation OFF


#define RCC_APB2ENR_TIM1EN                   0x00000800               //TIM1 Timer clock enable

#define TIM_CCMR1_OC1M                       0x00000070               //OC1M[2:0] bits (Output Compare 1 Mode)
#define TIM_CCMR1_OC1M_0                     0x00000010
#define TIM_CCMR1_OC1M_1                     0x00000020
#define TIM_CCMR1_OC1M_2                     0x00000040

#define TIM_CCMR1_OC1PE                      0x00000008               //Output Compare 1 Preload enable

#define TIM_CCER_CC1E                        0x00000001               //Capture/Compare 1 output enable
#define TIM_CCER_CC1P                        0x00000002               //Capture/Compare 1 output Polarity


#define TIM_CR2_MMS                          0x00000070               //MMS[2:0] bits (Master Mode Selection)
#define TIM_CR2_MMS_0                        0x00000010
#define TIM_CR2_MMS_1                        0x00000020
#define TIM_CR2_MMS_2                        0x00000040

#define TIM_CR1_CEN                          0x00000001                //Counter enable

#define TIM_CR1_ARPE                         0x00000080                //Auto-reload preload enable

#define TIM_BDTR_MOE                         0x00008000                //Main Output enable

#define TIM_DIER_UIE                         0x00000001                //Update interrupt enable


#define SPI_CR1_CPHA                        0x00000001                 //Clock Phase
#define SPI_CR1_CPOL                        0x00000002                 //Clock Polarity
#define SPI_CR1_MSTR                        0x00000004                 //Master Selection

#define SPI_CR1_BR                          0x00000038                 //BR[2:0] bits (Baud Rate Control)
#define SPI_CR1_BR_0                        0x00000008
#define SPI_CR1_BR_1                        0x00000010
#define SPI_CR1_BR_2                        0x00000020

#define SPI_CR1_BR_DIV2                     0x00000000                 //Prescaler bits for divide by 2
#define SPI_CR1_BR_DIV4                     0x00000008                 //Prescaler bits for divide by 4
#define SPI_CR1_BR_DIV8                     0x00000010                 //Prescaler bits for divide by 8
#define SPI_CR1_BR_DIV16                    0x00000018                 //Prescaler bits for divide by 16
#define SPI_CR1_BR_DIV32                    0x00000020                 //Prescaler bits for divide by 32
#define SPI_CR1_BR_DIV64                    0x00000028                 //Prescaler bits for divide by 64
#define SPI_CR1_BR_DIV128                   0x00000030                 //Prescaler bits for divide by 128
#define SPI_CR1_BR_DIV256                   0x00000038                 //Prescaler bits for divide by 256


#define SPI_CR1_SPE                         0x00000040                 //SPI Enable
#define SPI_CR1_LSBFIRST                    0x00000080                 //Frame Format
#define SPI_CR1_SSI                         0x00000100                 //Internal slave select
#define SPI_CR1_SSM                         0x00000200                 //Software slave management
#define SPI_CR1_RXONLY                      0x00000400                 //Receive only
#define SPI_CR1_DFF                         0x00000800                 //Data Frame Format
#define SPI_CR1_CRCNEXT                     0x00001000                 //Transmit CRC next
#define SPI_CR1_CRCEN                       0x00002000                 //Hardware CRC calculation enable
#define SPI_CR1_BIDIOE                      0x00004000                 //Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE                    0x00008000                 //Bidirectional data mode enable

#define SPI_CR2_RXDMAEN                     0x00000001                 //Rx Buffer DMA Enable
#define SPI_CR2_TXDMAEN                     0x00000002                 //Tx Buffer DMA Enable
#define SPI_CR2_SSOE                        0x00000004                 //SS Output Enable
#define SPI_CR2_ERRIE                       0x00000020                 //Error Interrupt Enable
#define SPI_CR2_RXNEIE                      0x00000040                 //RX buffer Not Empty Interrupt Enable
#define SPI_CR2_TXEIE                       0x00000080                 //Tx buffer Empty Interrupt Enable

#define SPI_SR_RXNE                         0x00000001                 //Receive buffer Not Empty
#define SPI_SR_TXE                          0x00000002                 //Transmit buffer Empty
#define SPI_SR_CHSIDE                       0x00000004                 //Channel side
#define SPI_SR_UDR                          0x00000008                 //Underrun flag
#define SPI_SR_CRCERR                       0x00000010                 //CRC Error flag
#define SPI_SR_MODF                         0x00000020                 //Mode fault
#define SPI_SR_OVR                          0x00000040                 //Overrun flag
#define SPI_SR_BSY                          0x00000080                 //Busy flag


#define USART_SR_TXE                        0x00000080                 //Transmit Data Register Empty


#define USART_CR1_SBK                       0x00000001                 //Send Break
#define USART_CR1_RWU                       0x00000002                 //Receiver wakeup
#define USART_CR1_RE                        0x00000004                 //Receiver Enable
#define USART_CR1_TE                        0x00000008                 //Transmitter Enable
#define USART_CR1_IDLEIE                    0x00000010                 //IDLE Interrupt Enable
#define USART_CR1_RXNEIE                    0x00000020                 //RXNE Interrupt Enable
#define USART_CR1_TCIE                      0x00000040                 //Transmission Complete Interrupt Enable
#define USART_CR1_TXEIE                     0x00000080                 //PE Interrupt Enable
#define USART_CR1_PEIE                      0x00000100                 //PE Interrupt Enable
#define USART_CR1_PS                        0x00000200                 //Parity Selection
#define USART_CR1_PCE                       0x00000400                 //Parity Control Enable
#define USART_CR1_WAKE                      0x00000800                 //Wakeup method
#define USART_CR1_M                         0x00001000                 //Word length
#define USART_CR1_UE                        0x00002000                 //USART Enable

#define USART_CR3_DMAR                      0x00000040                 //DMA Enable Receiver
#define USART_CR3_DMAT                      0x00000080                 //DMA Enable Transmitter


//DMA definitions
#define DMA_CCR_EN                  0x00000001                            //Channel enable
#define DMA_CCR_TCIE                0x00000002                            //Transfer complete interrupt enable
#define DMA_CCR_HTIE                0x00000004                            //Half Transfer interrupt enable
#define DMA_CCR_TEIE                0x00000008                            //Transfer error interrupt enable
#define DMA_CCR_DIR                 0x00000010                            //Data transfer direction
#define DMA_CCR_CIRC                0x00000020                            //Circular mode
#define DMA_CCR_PINC                0x00000040                            //Peripheral increment mode
#define DMA_CCR_MINC                0x00000080                            //Memory increment mode

#define DMA_CCR_PSIZE               0x00000300                            //PSIZE[1:0] bits (Peripheral size)
#define DMA_CCR_PSIZE_0             0x00000100
#define DMA_CCR_PSIZE_1             0x00000200

#define DMA_CCR_MSIZE               0x00000C00                            //MSIZE[1:0] bits (Memory size)
#define DMA_CCR_MSIZE_0             0x00000400
#define DMA_CCR_MSIZE_1             0x00000800

#define DMA_CCR_PL                  0x00003000                            //PL[1:0] bits(Channel Priority level)
#define DMA_CCR_PL_0                0x00001000
#define DMA_CCR_PL_1                0x00002000

#define DMA_CCR_MEM2MEM             0x00004000                            //Memory to memory mode

#define DMA_ISR_TCIF3                        0x00000200                //Channel 3 Transfer Complete flag
#define DMA_ISR_HTIF3                        0x00000400                //Channel 3 Half Transfer flag

#define DMA_ISR_TCIF5                        0x00020000                //Channel 5 Transfer Complete flag
#define DMA_ISR_HTIF5                        0x00040000                //Channel 5 Half Transfer flag


#define DMA_IFCR_CGIF3                       0x00000100                //Channel 3 Global interrupt clear
#define DMA_IFCR_CTCIF3                      0x00000200                //Channel 3 Transfer Complete clear
#define DMA_IFCR_CHTIF3                      0x00000400                //Channel 3 Half Transfer clear
#define DMA_IFCR_CTEIF3                      0x00000800                //Channel 3 Transfer Error clear

#define DMA_IFCR_CGIF5                       0x00010000                //Channel 5 Global interrupt clear
#define DMA_IFCR_CTCIF5                      0x00020000                //Channel 5 Transfer Complete clear
#define DMA_IFCR_CHTIF5                      0x00040000                //Channel 5 Half Transfer clear
#define DMA_IFCR_CTEIF5                      0x00080000                //Channel 5 Transfer Error clear


#define AFIO_MAPR_TIM3_REMAP_1               0x00000800                //Timer3 pins on alternate function pins

#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        0x02000000                //JTAG-DP Disabled and SW-DP Enabled


#define AIRCR_VECTKEY_MASK      ((uint32_t)0x05FA0000)

#define EXTI_Line18             ((uint32_t)0x40000)  //External interrupt line 18 Connected to the USB Device/USB OTG FS

#define NVIC_PriorityGroup_2    ((uint32_t)0x500)    //2 bits for pre-emption priority, 2 bits for subpriority


#define USB_CNTR_SOFM                           0x00000200              //Start Of Frame Interrupt Mask
#define USB_CNTR_RESETM                         0x00000400              //RESET Interrupt Mask
#define USB_CNTR_SUSPM                          0x00000800              //Suspend mode Interrupt Mask
#define USB_CNTR_CTRM                           0x00008000              //Correct Transfer Interrupt Mask

#define USB_EP_TX_VALID                         0x00000030              //EndPoint TX VALID
#define USB_EP_TX_NAK                           0x00000020              //EndPoint TX NAKed

#define USB_EP_CTR_RX                           0x00008000              //EndPoint Correct TRansfer RX
#define USB_EPRX_STAT                           0x00003000              //EndPoint RX STATus bit field
#define USB_EP_SETUP                            0x00000800              //EndPoint SETUP
#define USB_EP_T_FIELD                          0x00000600              //EndPoint TYPE
#define USB_EP_KIND                             0x00000100              //EndPoint KIND
#define USB_EP_CTR_TX                           0x00000080              //EndPoint Correct TRansfer TX
#define USB_EPTX_STAT                           0x00000030              //EndPoint TX STATus bit field
#define USB_EPADDR_FIELD                        0x0000000F              //EndPoint ADDRess FIELD

#define  USB_EPREG_MASK                      (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)

#define USB_ISTR_EP_ID                          0x0000000F               //Endpoint Identifier
#define USB_ISTR_ESOF                           0x00000100               //Expected Start Of Frame
#define USB_ISTR_SOF                            0x00000200               //Start Of Frame
#define USB_ISTR_RESET                          0x00000400               //USB RESET request
#define USB_ISTR_SUSP                           0x00000800               //Suspend mode request
#define USB_ISTR_WKUP                           0x00001000               //Wake up
#define USB_ISTR_ERR                            0x00002000               //Error
#define USB_ISTR_PMAOVR                         0x00004000               //Packet Memory Area Over / Underrun
#define USB_ISTR_CTR                            0x00008000               //Correct Transfer

#define USB_EP_BULK                             0x00000000               //EndPoint BULK
#define USB_EP_CONTROL                          0x00000200               //EndPoint CONTROL
#define USB_EP_INTERRUPT                        0x00000600               //EndPoint INTERRUPT

#define USB_EP_RX_DIS                           0x00000000               //EndPoint RX DISabled
#define USB_EP_RX_VALID                         0x00003000               //EndPoint RX VALID

#define USB_DADDR_EF                            0x00000080               //Enable Function


//I2C settings
#define I2C_CR1_PE                          0x00000001                   //Peripheral Enable
#define I2C_CR1_SMBUS                       0x00000002                   //SMBus Mode
#define I2C_CR1_SMBTYPE                     0x00000008                   //SMBus Type
#define I2C_CR1_ENARP                       0x00000010                   //ARP Enable
#define I2C_CR1_ENPEC                       0x00000020                   //PEC Enable
#define I2C_CR1_ENGC                        0x00000040                   //General Call Enable
#define I2C_CR1_NOSTRETCH                   0x00000080                   //Clock Stretching Disable (Slave mode)
#define I2C_CR1_START                       0x00000100                   //Start Generation
#define I2C_CR1_STOP                        0x00000200                   //Stop Generation
#define I2C_CR1_ACK                         0x00000400                   //Acknowledge Enable
#define I2C_CR1_POS                         0x00000800                   //Acknowledge/PEC Position (for data reception)
#define I2C_CR1_PEC                         0x00001000                   //Packet Error Checking
#define I2C_CR1_ALERT                       0x00002000                   //SMBus Alert
#define I2C_CR1_SWRST                       0x00008000                   //Software Reset

#define I2C_CR2_FREQ_32MHZ                  0x00000020

#define I2C_CR2_FREQ_0                      0x00000001                   //FREQ[5:0] bits (Peripheral Clock Frequency)
#define I2C_CR2_FREQ_1                      0x00000002
#define I2C_CR2_FREQ_2                      0x00000004
#define I2C_CR2_FREQ_3                      0x00000008
#define I2C_CR2_FREQ_4                      0x00000010
#define I2C_CR2_FREQ_5                      0x00000020

#define I2C_CR2_ITERREN                     0x00000100                   //Error Interrupt Enable
#define I2C_CR2_ITEVTEN                     0x00000200                   //Event Interrupt Enable
#define I2C_CR2_ITBUFEN                     0x00000400                   //Buffer Interrupt Enable
#define I2C_CR2_DMAEN                       0x00000800                   //DMA Requests Enable
#define I2C_CR2_LAST                        0x00001000                   //DMA Last Transfer


#define I2C_OAR1_ADD0                       0x00000001                   //Bit 0
#define I2C_OAR1_ADD1                       0x00000002                   //Bit 1
#define I2C_OAR1_ADD2                       0x00000004                   //Bit 2
#define I2C_OAR1_ADD3                       0x00000008                   //Bit 3
#define I2C_OAR1_ADD4                       0x00000010                   //Bit 4
#define I2C_OAR1_ADD5                       0x00000020                   //Bit 5
#define I2C_OAR1_ADD6                       0x00000040                   //Bit 6
#define I2C_OAR1_ADD7                       0x00000080                   //Bit 7
#define I2C_OAR1_ADD8                       0x00000100                   //Bit 8
#define I2C_OAR1_ADD9                       0x00000200                   //Bit 9

#define I2C_OAR1_ADDMODE                    0x00008000                   //Addressing Mode (Slave mode)


#define I2C_OAR2_ENDUAL                     0x00000001                   //Dual addressing mode enable
#define I2C_OAR2_ADD2                       0x000000FE                   //Interface address


#define I2C_SR1_SB                          0x00000001                   //Start Bit (Master mode)
#define I2C_SR1_ADDR                        0x00000002                   //Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF                         0x00000004                   //Byte Transfer Finished
#define I2C_SR1_ADD10                       0x00000008                   //10-bit header sent (Master mode)
#define I2C_SR1_STOPF                       0x00000010                   //Stop detection (Slave mode)
#define I2C_SR1_RXNE                        0x00000040                   //Data Register not Empty (receivers)
#define I2C_SR1_TXE                         0x00000080                   //Data Register Empty (transmitters)
#define I2C_SR1_BERR                        0x00000100                   //Bus Error
#define I2C_SR1_ARLO                        0x00000200                   //Arbitration Lost (master mode)
#define I2C_SR1_AF                          0x00000400                   //Acknowledge Failure
#define I2C_SR1_OVR                         0x00000800                   //Overrun/Underrun
#define I2C_SR1_PECERR                      0x00001000                   //PEC Error in reception
#define I2C_SR1_TIMEOUT                     0x00004000                   //Timeout or Tlow Error
#define I2C_SR1_SMBALERT                    0x00008000                   //SMBus Alert


#define I2C_SR2_MSL                         0x00000001                   //Master/Slave
#define I2C_SR2_BUSY                        0x00000002                   //Bus Busy
#define I2C_SR2_TRA                         0x00000004                   //Transmitter/Receiver
#define I2C_SR2_GENCALL                     0x00000010                   //General Call Address (Slave mode)
#define I2C_SR2_SMBDEFAULT                  0x00000020                   //SMBus Device Default Address (Slave mode)
#define I2C_SR2_SMBHOST                     0x00000040                   //SMBus Host Header (Slave mode)
#define I2C_SR2_DUALF                       0x00000080                   //Dual Flag (Slave mode)
#define I2C_SR2_PEC                         0x0000FF00                   //Packet Error Checking Register


#define I2C_CCR_CCR                         0x00000FFF                   //Clock Control Register in Fast/Standard mode (Master mode)
#define I2C_CCR_DUTY                        0x00004000                   //Fast Mode Duty Cycle
#define I2C_CCR_FS                          0x00008000                   //I2C Master Mode Selection

#define I2C_CCR_32MHZ_100KHZ                0x000000A0                   //Obtain 100KHz SCL when PCLK1 is 32MHz

//Defines for general purpose I/O pin mode to set as input or to set the speed grade for outputs
#define GPIO_MODE_INPUT  0x00000000
#define GPIO_MODE_2MHZ   0x00000002
#define GPIO_MODE_10MHZ  0x00000001
#define GPIO_MODE_50MHZ  0x00000003

//Defines for general purpose output pin configuration to set as either push pull or open drain for actual output pin or alternate function pin
#define GPIO_CONF_O_PP   0x00000000
#define GPIO_CONF_O_OD   0x00000001
#define GPIO_CONF_AF_PP  0x00000002
#define GPIO_CONF_AF_OD  0x00000003

//Defines for general purpose input pin configuration to set as analog input, floating input or with pull up/pull down resistor input.
#define GPIO_CONF_I_ANA  0x00000000
#define GPIO_CONF_I_FLT  0x00000001
#define GPIO_CONF_I_PUPD 0x00000002


#endif


