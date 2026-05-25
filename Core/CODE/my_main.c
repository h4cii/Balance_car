
#include "my_main.h"

#include "balance_control.h"

void my_main(void)
{
  balance_control_init();

  for (;;)
  {
    balance_control_poll();
  }
}

