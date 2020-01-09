#include "mainpp.h"
#include "main.h"
#include <stdio.h>

void setup() {
  printf("Hello, World!\r\n");
}

int i = 0;
void loop() {
  printf("Hello %d\r\n", i++);
  HAL_Delay(1000);
}
