//
// Created by root on 22-9-22.
//

#include <gtest/gtest.h>
#include "uart.h"

int add(int lhs, int rhs) { return lhs + rhs; }

unsigned char led_on[] = {0Xdd, 0X05, 0X24, 0X00, 0X00};//开灯
unsigned char led_off[] = {0xdd, 0x05, 0x24, 0x00, 0x01};//关灯
unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//报警
unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//关报警

TEST(FactorialTest, Negative) {
  // open_uart();
  main_uart();
}

TEST(FactorialTest, DOOR) {
  int fd = open_uart();
  init_uart(fd);
  //门禁
  write_uart_data(fd, alert_off);
}

int main(int argc, char **argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}