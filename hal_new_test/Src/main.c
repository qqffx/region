#include "main.h"
#include "stm32h7xx_hal.h"
#include "eth.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define TIM2_CCR1_Address ((unsigned int)0x40000000 + 0x34)
#define DMA_BUFFER_SIZE   61440
#define SIZE 512
#define Destination_Address ((unsigned int)0x30000000)

USBD_HandleTypeDef hUsbDeviceFS;

// TODO: why does this not zeroed array 
unsigned int buffer[DMA_BUFFER_SIZE] __attribute__((section(".orbita_buffer")));
unsigned int bufferIndex = 0;
unsigned int bufferValue = 0;


typedef struct {
  unsigned int NDTR;
  unsigned int circBufferCounter;
  unsigned int dmaFlag;
  unsigned int StartBufferEntry;
  
} DEBUG_STRUCT;

#define debugBufferLength 4
DEBUG_STRUCT debugBuffer[debugBufferLength];
unsigned int debugIndex = 0;

volatile unsigned int dmaFlag = 0;
volatile unsigned int flipFlag = 0;

char string[22] = "end\r\n";

unsigned int transmissionFlag = 0;

USBD_HandleTypeDef *usbDevice;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

static void my_initClocks(void);
static void my_initGPIO(void);
static void my_DMA_init(void);
static void my_TIM2_initInputCaptureTimer(void);
static void TIM6_myInit(void);

static void printArray(void);
static void zeroArray(void);
static void printDebugBuffer(void);
static void writeFile(void);

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

  HAL_Init();
  
  //SCB_DisableICache();
  //SCB_DisableDCache();
  SCB_InvalidateICache();
  SCB_InvalidateDCache();
  SCB_EnableICache();
  SCB_EnableDCache();

  my_initClocks();
  my_initGPIO();
  
  MX_ETH_Init();
  MX_USART3_UART_Init();
  usbDevice = MX_USB_DEVICE_Init();
  
  zeroArray();
  
  HAL_Delay(2000);  
  printf("waiting\r\n");  
  while((strcmp(string, "send_data")) != 0);
  printf("start \r\n");  
  
  my_DMA_init();
  my_TIM2_initInputCaptureTimer();
  //TIM7_myInit();
  TIM6_myInit();

  
  unsigned int circ_buffer_ptr = 0;
  unsigned int counter = 0;
  unsigned int stopFlag = 0;
  unsigned int firstHalfDone = 0;
  while (1)
  {
    if(stopFlag < 20) {
      if(DMA1_Stream0->NDTR < DMA_BUFFER_SIZE/2) {
        
        for(int i = 0; i < DMA_BUFFER_SIZE/SIZE/2; ++i) {
          //SCB_InvalidateDCache();
          CDC_Transmit_FS(buffer+SIZE*i, SIZE*sizeof(unsigned int));
          while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState != 0);
        } // transfer via USB first half of buffer
        
        while(!flipFlag);
        
        for(int i = DMA_BUFFER_SIZE/SIZE/2; i < DMA_BUFFER_SIZE/SIZE; ++i) {
          //SCB_InvalidateDCache();
          CDC_Transmit_FS(buffer+SIZE*i, SIZE*sizeof(unsigned int));
          while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState != 0);
        } // transfer via USB first half of buffer
        stopFlag++;
        
      }
    }
    
    if(stopFlag == 20) {
      buffer[0] = 0x00646E65;
      CDC_Transmit_FS(buffer, SIZE*sizeof(unsigned int));
      while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState != 0);
    }
      
    
  }// while
}// main

void TIM2_IRQHandler(void) {
  
  // reset interrupt flag
  TIM2->SR = 0;
  //GPIOB->ODR ^= (0x1 << 7);
  
  //tick = systick_ms;
  //flag++;
}

void DMA1_Stream0_IRQHandler(void) {
  
  // clear interrupt flag
  DMA1->LIFCR |= (0x1 << 3) | (0x1 << 5);
  //TIM6->CR1 &= ~(0x1);
  GPIOB->ODR |= (0x1 << 7);
  dmaFlag++;
  flipFlag++;
  
}

void TIM6_DAC_IRQHandler(void) {
  // reset interrupt flag
  TIM6->SR &= ~(0x1 << 0);
  
  //buffer[bufferIndex] = TIM2->CNT;
  //bufferValue += 5;
  //bufferIndex = (++bufferIndex)%DMA_BUFFER_SIZE;
  // toggle pin
  GPIOG->ODR ^= (0x1 << 0);
}


static void my_initGPIO(void) {
  // GPIO initialization
  // WARNING idk why but if deactivate some of them USB stop work, even if
  // designated pins (overcurrent\power_switch) still active
  RCC->AHB4ENR |= (1 << 0);
  RCC->AHB4ENR |= (1 << 1);
  RCC->AHB4ENR |= (1 << 2);
  RCC->AHB4ENR |= (1 << 3);
  RCC->AHB4ENR |= (1 << 4);
  RCC->AHB4ENR |= (1 << 6);
  RCC->AHB4ENR |= (1 << 7);  
  // configurate Blue_LD pin
  GPIOB->MODER &= ~(0x2 << 14);
  // configurate Red_LD pin
  GPIOB->MODER &= ~(0x2 << 28);
  // configurate toggle pin for INP/CAP
  GPIOG->MODER &= ~(0x2 << 0);
  // configurate external LD pin
  GPIOE->MODER &= ~(0x2 << 0);
  // Usb powerSwitch ON pin (reset)
  GPIOG->ODR &= ~(0x1 << 6);
  // configure usb power switch on PIT
  GPIOG->MODER &= ~(0x2 << 12);
  GPIOG->PUPDR &= ~(0x3 << 12);
  GPIOG->OSPEEDR &= ~(0x3 << 12);
  // configure usb overcurrent pin
  GPIOG->MODER &= ~(0x3 << 14);
  GPIOG->PUPDR &= ~(0x3 << 14);
  // deactivate pin
  GPIOG->ODR &= ~(0x1 << 0);
  GPIOE->ODR &= ~(0x1 << 0);
  // NOTE: config PA5-leg as input for InpCap TIM2
  // clear PA5 part of MODER register
  GPIOA->MODER &= ~(0x3 << 10);
  // put PA5 in alternate function mode
  GPIOA->MODER |= (0x2 << 10);
  // config PA5 for using (AF1 - TIM2_CH1)
  GPIOA->AFR[0] |= (0x1 << 20);
  // config PA5 (nopull)
  GPIOA->PUPDR &= ~(0x3 << 10);
  // config PA5 (VERY HIGH FREQ)
  GPIOA->OSPEEDR |= (0x3 << 10);
  // NOTE: config PC13-leg as external interrupt for user button (blue button)
  // enable clock to SysCFG
  RCC->APB4ENR |= (0x1 << 1);
  // config pin as input
  GPIOC->MODER &= ~(0x3 << 26);
  // set PC13 pin as source of External Interrupt
  SYSCFG->EXTICR[3] |= (0x2 << 4);
  // enable interrupt generation to PC13 leg via masking
  EXTI_D1->IMR1 |= (0x1 << 13);
  // use (Rising edge) for PC13 event generation -> clear Falling edge register
  EXTI->FTSR1 &= ~(0x1 << 13);
  // use (Rising edge) for PC13
  EXTI->RTSR1 |= (0x1 << 13);
  // config PC13 (NOPULL)
  GPIOC->PUPDR &= ~(0x3 << 26);
}

static void my_TIM2_initInputCaptureTimer(void) {
  // enable clock source for timer
  RCC->APB1LENR |= (0x1 << 0);
  // set prescaler to 200
  TIM2->PSC = 1 - 1;
  // choose TIM2_CH1 input
  TIM2->TISEL |= (0x0 << 0);
  // set channel 1 as input mapped on TI1
  TIM2->CCMR1 |= (0x1 << 0);
  // digital filter length (0)
  TIM2->CCMR1 |= (0x0 << 4);
  // rising & falling edge
  TIM2->CCER |= (0x1 << 1);
  TIM2->CCER |= (0x1 << 3);
  // prescaler to (0)
  TIM2->CCMR1 |= (0x0 << 2);
  // enable DMA interrupt &  disable Capture/Compare interrupt
  TIM2->DIER |= (0x1 << 9) | (0x0 << 1);
  // capture enabled
  TIM2->CCER |= (0x1 << 0);
  // reset registers WARNING: need for preloading PSC  
  TIM2->EGR |= (0x1 << 0);
  // enable TIM3 timer
  TIM2->CR1 |= TIM_CR1_CEN;
  // enable interrupt request
  // NVIC_EnableIRQ(TIM2_IRQn);
  // set priority
  // NVIC_SetPriority(TIM2_IRQn, 1);
}

static void my_DMA_init(void) {
  // enable DMA1 clocking
  RCC->AHB1ENR |= (0x1 << 0);
  // clear EN bit to 0
  DMA1_Stream0->CR &= ~(0x1 << 0);
  // safeguard EN bit reset
  while (DMA1_Stream0->CR & 0x1);
  // check LISR HISR registers
  if ((DMA1->HISR == 0) && (DMA1->LISR == 0))
    printf("status registers is clear\r\n");
  else
    printf("status register is not clear -- DMA wont start\r\n");
  // set peripheral addres
  DMA1_Stream0->PAR = TIM2_CCR1_Address;
  // set memory addres
  //DMA1_Stream0->M0AR = Destination_Address;
  DMA1_Stream0->M0AR = (unsigned int)buffer;
  // set total number of data items
  DMA1_Stream0->NDTR = DMA_BUFFER_SIZE;
  // NOTE: configurate TIM2_CH1 interrupt route
  // set DMAMUX to route request (TIM2_CH1)
  DMAMUX1_Channel0->CCR |= 18U;
  // set DMA priority (very high)
  DMA1_Stream0->CR |= (0x3 << 16);
  // set memory data size (32)
  DMA1_Stream0->CR |= (0x2 << 13);
  // set peripheral data size (32)
  DMA1_Stream0->CR |= (0x2 << 11);
  // set memory addres increment  (enable)
  DMA1_Stream0->CR |= (0x1 << 10);
  // set peripheral addres increment (disable)
  DMA1_Stream0->CR |= (0x0 << 9);
  // set circular buffer mode (enable)
  DMA1_Stream0->CR |= (0x1 << 8);
  // set data transfer direction (peripheral to memory)
  DMA1_Stream0->CR |= (0x0 << 6);
  // set transfer complete interrupt
  DMA1_Stream0->CR |= (0x1 << 4);
  // set transfer error interrupt
  DMA1_Stream0->CR |= (0x1 << 2);
  // enable DMA1
  DMA1_Stream0->CR |= (0x1 << 0);
  // enable IRQ
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
  printf("DMA1_Stream0 %lu \r\n", (DMA1_Stream0->CR & 0x1));
  printf("DMA1_Stream0 CIRC %lu \r\n", (DMA1_Stream0->CR >> 8) & 0x1);
}


// this part configurate TIM6 timer interrupt (TIM6_DAC_IRQHandler)
static void TIM6_myInit(void) {
  //printf("inside init \r\n");
  // start sending CLOCK to TIM6
  RCC->APB1LENR |= (1 << 4);
  // value timer count must reach before interrupt
  // now it should be approx (1 sec) (200Mhz apb1-timer clock / PSC / ARR = 1 Hz)
  // WARNING: -1 bcoz of the counter architecture -> he make 0 to 10000 same for PSC
  TIM6->ARR = 10 - 1;
  // prescaler clock that divide APB1 timers clock
  TIM6->PSC = 25 - 1;
  // enable interrupt
  TIM6->DIER |= TIM_DIER_UIE | TIM_DIER_UDE;
   // enable interrupt request
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  // set priority 
  NVIC_SetPriority(TIM6_DAC_IRQn, 0);
  // enable TIM6
  TIM6->CR1 |= TIM_CR1_CEN;
}


// just clear array for easier visualization
static void zeroArray(void) {
  for(int i = 0; i < DMA_BUFFER_SIZE; ++i) {
    buffer[i] = i;
  }
}

static void printDebugBuffer(void) {
  printf("Debug Info: \r\n");
  for(int i = 0; i < debugBufferLength; ++i) {
    
    if(!(i % 2)) {
      printf("------------------------------ \r\n");
    }
    printf("NDTR: %u circBuffer %u dmaFlag: %u StartBufferEntry %10u \r\n",
           debugBuffer[i].NDTR,
           debugBuffer[i].circBufferCounter,
           debugBuffer[i].dmaFlag,
           debugBuffer[i].StartBufferEntry);
  }
  //printf("end \r\n");
}

// print array in matrix like style
static void printArray(void) {
  printf("buffer: \r\n");
  for(int i = 0; i < 1000; ++i) {
    printf(" %12u ", buffer[i]);
    // new line every 10 outputs
    if(!((i+1) % 4)) {
      printf("\r\n");
    }
  }  
}

static void writeFile(void) {
  unsigned int circ_buffer_ptr = 0;
  for(int i = 0; i < (DMA_BUFFER_SIZE/SIZE); ++i) {
    CDC_Transmit_FS(buffer + circ_buffer_ptr, SIZE*sizeof(unsigned int));
    circ_buffer_ptr = (circ_buffer_ptr + SIZE) % DMA_BUFFER_SIZE;
    while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState != 0);
  }
}// writeFile
  


/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart3, &c[0], 1, 10);
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USER CODE BEGIN 7 */
// this config should get (400 Mhz) CPU clock from (8 Mhz Hse) RC oscillator
void my_initClocks(void) {
  // WARNING: this is needed bcoz of high CPU freq
  // set up power Level Scale 1
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  PWR->D3CR |= (0x3 << 14);
  // power level configuration safeguard
  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY);
  // Enable HSI48 for usb clocking
  RCC->CR |= (0x1 << 12);
  // Enable HSE as Bypass RC-oscillator
  // HSEBYP bit enabled
  RCC->CR |= (1 << 18);
  // HSEON bit enabled
  RCC->CR |= (1 << 16);
  // safeguard HSE
  while(!(RCC->CR & (0x1 << 17)));
  // NOTE: for further configuration
  // PLL OFF 
  RCC->CR &= ~(RCC_CR_PLL1ON);
  // set PLL1 source as (HSE)
  RCC->PLLCKSELR |= (0x2 << 0);
  // set PLL1 prescaler to (1)
  RCC->PLLCKSELR |= (0x1 << 4);
  // set PLL1 multiplier to (100)
  RCC->PLL1DIVR |= (0x64 << 0);
  // set PLL1 DIVP1 to (2)
  RCC->PLL1DIVR |= (0x1 << 9);
  // set PLL1 DIVQ1 to (4)
  RCC->PLL1DIVR |= (0x3 << 16);
  // set PLL1 DIVR1 to (2)
  RCC->PLL1DIVR |= (0x1 << 24);
  // disable PLL1 FRACTION mode 
  RCC->PLLCFGR &= ~(0x1 << 0);
  // set FRACN1 to (0)
  RCC->PLL1FRACR = 0x0;
  // set PLL input freq range to (8 - 16 Mhz)
  RCC->PLLCFGR |= (0x3 << 2);
  // set PLL VCO frequency range (wide -> 192 - 836 Mhz)
  RCC->PLLCFGR |= (0x1 << 1);
  // set PLL1_DIVP|DIVQ|DIVR enable
  RCC->PLLCFGR |= (0x1 << 18) | (0x1 << 17) | (0x1 << 16);
  // enable PLL1 FRACTION mode 
  RCC->PLLCFGR |= (0x1 << 0);
  // enable PLL1
  RCC->CR |= (RCC_CR_PLL1ON);
  // safeguard PLL1
  while(!(RCC->CR & (0x1 << 25)));
  // WARNING should be matched with CPU speed 
  // set FLASH Latency (wait 2) 
  FLASH->ACR |= (0x2 << 0);
  // NOTE 0x8 -> 2, 0x9 -> 4 ... first bit of reg part should be 1
  // set HPRE to (2) 
  RCC->D1CFGR |= (0x8 << 0);
  // set D1CPRE to (1)
  RCC->D1CFGR |= (0x0 << 8);
  // NOTE: actually do not needed
  // safeguard PLL1  
  while(!(RCC->CR & (0x1 << 25)));
  // set D1PPRE to (2)
  RCC->D1CFGR |= (0x4 << 4);
  // set D2PPRE1 to (2)
  RCC->D2CFGR |= (0x4 << 4);
  // set D2PPRE2 to (2)
  RCC->D2CFGR |= (0x4 << 8);
  // set D3PPRE2 to (2)
  RCC->D3CFGR |= (0x4 << 4);
  // set PLL1 as (sysClk)
  RCC->CFGR |= (0x3 << 0);
  // set USART2/3/4/5/7/8 clockw as (rcc_pclk1)
  RCC->D2CCIP2R &= ~(0x7 << 0);
  RCC->D2CCIP2R |= (0x0 << 0);
  // set USB clock as (HSI48)
  RCC->D2CCIP2R &= ~(0x3 << 20);
  RCC->D2CCIP2R |= (0x3 << 20);
  // TODO make my own realization of this func
  // NOTE 1 us is an overkill and just busting CPU for no real purpose
  //SysTick_Config(400000000/1000);
  // set prirority of the SysTick
  //NVIC_SetPriority(SysTick_IRQn, 0);
}
/* USER CODE END 7 */



