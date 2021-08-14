#include <cstdio>
#include <chrono>
#include <sys/time.h>

#include "peripherals/stm32f4xx/gpio.hpp"
#include "utility/time/time.hpp"
#include "utility/log.hpp"

extern "C"
{
int gettimeofday( struct timeval *tv, void *tzvp )
{
    uint64_t t = 1000'0001'4356;  // get uptime in nanoseconds
    tv->tv_sec = t / 1000000000;  // convert to seconds
    tv->tv_usec = ( t % 1000000000 ) / 1000;  // get remaining microseconds
    return 0;  // return non-zero for error
}
}

int main()
{
  sjsu::LogInfo("Starting FK407M1 Gpio Application...");
  ///////////// Setup LED GPIO /////////////
  sjsu::stm32f4xx::Gpio & led = sjsu::stm32f4xx::GetGpio<'C', 13>();
  led.Initialize();
  led.SetAsOutput();

  auto system_start = std::chrono::system_clock::now();
  // settimeofday();
  auto steady_start = std::chrono::steady_clock::now();

  sjsu::stm32f4xx::Gpio & button = sjsu::stm32f4xx::GetGpio<'A', 15>();
  button.GetPin().settings.PullUp();
  button.Initialize();
  button.SetAsInput();

  while (true)
  {
    // If button is not pressed (true), repeat blinking led
    // If the button is held down (false), stop blinking.
    if (button.Read())
    {
      led.SetLow();
      sjsu::Delay(100ms);

      led.SetHigh();
      sjsu::Delay(100ms);
    }
    else
    {
      led.SetHigh();
    }
  }
  return 0;
}
