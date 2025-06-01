#include "logging.h"

#ifdef DEBUG

int debug_print(const char *fmt, ...)
{
  char buffer[128]; // Adjust size as needed
  va_list args;
  va_start(args, fmt);

  // Format the string
  int result = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (result < 0)
  {
    const char *err_msg = "vsnprintf error\n";
    // Handle snprintf error
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)err_msg, strlen(err_msg), 100);
    return result;
  }
  else
  {
    // Ensure null termination
    buffer[sizeof(buffer) - 1] = '\0';

    // Get the actual length
    size_t len = (size_t)result;
    if (len > sizeof(buffer))
    {
      len = sizeof(buffer) - 1;
    }

    // Transmit the formatted message
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)buffer, len, 100);
    return result;
  }
}
// Calling code should wrap in #ifdef DEBUG, so there is no #else here.
// We'd rather see a compile-time error to enforce the conditional.
#endif
