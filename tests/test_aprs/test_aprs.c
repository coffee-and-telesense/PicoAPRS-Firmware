

// Gets us the peripheral handles and pin definitions from cubemx generated code
#include "main.h"     // For HAL definitions and Error_Handler
#include "gpio.h"     // For GPIO functions
#include "i2c.h"      // For I2C functions
#include "usart.h"    // For UART functions
#include "aprs.h"
#include "ax25.h"

extern void SystemClock_Config(void); // For system clock configuration

// redirect printf
int _write(int file, char *ptr, int len)
{
    
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// input values
uint8_t analogValues[5][5] = {
    "001,",
    "002,",
    "003,",
    "004,",
    "005,"
};
char *digitalValue = "00001100";
char *comment = "hello!";

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  //MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Your test code here */
  while (1)
  {

    //create the aprs packet
    telemetryInfoFrame *tFrame = initTFrame();
    updateTelemData(tFrame, analogValues,(uint8_t*) digitalValue, comment);
    concatTelemData(tFrame);

    //wrap aprs packet in ax25 packet
    ax25Frame *frame = initFrame(tFrame->tData, tFrame->tDataSize);
    encodedAx25Frame encodedFrame = processFrameVerbose(frame);
    

    //clean up
    freeFrames(tFrame, NULL, NULL, NULL, NULL, &encodedFrame);
    
    //blinky
    HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
   
    HAL_Delay(10000);
    
  }
}