//
// Created by root on 22-9-22.
//

#ifndef SMART_UART_SRC_UART_H_
#define SMART_UART_SRC_UART_H_
int open_uart();
int init_uart(int fd);
int read_uart_data(int fd, char data[], char coll_data[3]);
int write_uart_data(int fd,unsigned char cmd[]);
int led(int fd, int light, int pic);
int temperature(int fd, int tmp);
int door(int fd, int pic, int close);
int humidity(int fd, int hum);
int tmp_hum_ctrl(int fd, int coll_data[]);
int main_uart();
#endif //SMART_UART_SRC_UART_H_
