#include <cstdint>

#include "utility/log.hpp"
#include "utility/time/time.hpp"

int main()
{
  sjsu::LogInfo("Starting Hello World Application");

  while (true)
  {
    for (uint8_t i = 0; i < 16; i++)
    {
      sjsu::LogInfo("Hello World 0x%X", i);
      sjsu::Delay(500ms);
      if (i == 5)
      {
        try
        {
          throw 5;
        }
        catch (...)
        {
          sjsu::LogInfo("Exception!");
        }
      }
    }
  }
  return -1;
}
