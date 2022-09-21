//
// Created by root on 22-9-22.
//
#include "uart.h"
#include "iostream"
#include <stdio.h> /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <string.h>
#include <fcntl.h> /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h> /*错误号定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <assert.h>
#include <poll.h>
using namespace std;

int main_uart() {
  char data[36] = {0};//接收数据
  char coll_data[3] = {0};//温度 湿度 亮度

  int fd = open_uart();
  init_uart(fd);
  while (1) {
    sleep(1);
    //读串口数据，温度 湿度 亮度按顺序存放coll_data
    // read_uart_data(fd, data, coll_data);
    /*if(door(fd,1,0)){
        led(fd,coll_data[2],0);
    }*/
    //门禁
//	door(fd,1,1);
    //湿度
    //humidity(fd,coll_data[1]);
    //路灯控制
    //led(fd,coll_data[2],0);
    //温度控制
    temperature(fd, coll_data[0]);
  }
  return 0;
}

//湿度控制
int humidity(int fd, int hum) {

  unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//报警
  unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//关报警
  unsigned char fan_on[] = {0xdd, 0x05, 0x24, 0x00, 0x04};//开风扇
  unsigned char fan_off[] = {0xdd, 0x05, 0x24, 0x00, 0x08};//关风扇
  //<35关警报、关风扇 35<hum<60开风扇  >30报警且开风扇
  if (hum <= 35) {
    write_uart_data(fd, alert_off);
    write_uart_data(fd, fan_off);
  } else if (hum > 35 && hum < 60) {
    write_uart_data(fd, alert_off);
    write_uart_data(fd, fan_on);
  } else {
    write_uart_data(fd, fan_on);
    write_uart_data(fd, alert_on);
  }
  return fd;
}

//门禁,pic存放图片信息,close存放客户端关门指令
int door(int fd, int pic, int close) {
  unsigned char door_on[] = {0xdd, 0x08, 0x24, 0x00, 0x09};//开门
  unsigned char door_off[] = {0xdd, 0x08, 0x24, 0x00, 0x0a};//关门
  if (pic) {
    //有权限的人脸开门
    if (pic == 1) {
      write_uart_data(fd, door_on);
      return 1;
    } else if (pic == 0) {    //没有权限人脸关门
      write_uart_data(fd, door_off);
      printf("您没有权限");
      return 0;
    }
  } else if (close) {//发送关门命令就关门
    write_uart_data(fd, door_off);
  } else {
    write_uart_data(fd, door_on);
  }
  return 0;
}

// ------
//灯光控制,pic存放人脸识别信息
//优先手动控制,open close接收客户端的指令
int led(int fd, int light, int pic) {
  unsigned char led_on[] = {0Xdd, 0X05, 0X24, 0X00, 0X00};//开灯
  unsigned char led_off[] = {0xdd, 0x05, 0x24, 0x00, 0x01};//关灯
  unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//报警
  unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//关报警
  //有人脸就手动开关
  if (pic) {
    /*if(pic==1){
        //有权限的人脸，手动开关
        if(open){
            write_uart_data(fd,led_on);
        }else if(close){
        write_uart_data(fd,led_off);
        }
    }else if(pic==0){
        //无权限人脸报警持续一段时间关闭
        write_uart_data(fd,alert_on);
        sleep(1000);
        write_uart_data(fd,alert_off);
    }*/
  } else {

    //自动开关灯，light<40开灯，light>80关灯
    if (light < 75) {
      write_uart_data(fd, led_on);
    } else if (light >= 75) {
      write_uart_data(fd, led_off);
    }

  }
  return fd;
}

//温度控制
int temperature(int fd, int tmp) {
  unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//报警
  unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//关报警
  unsigned char fan_on[] = {0xdd, 0x05, 0x24, 0x00, 0x04};//开风扇
  unsigned char fan_off[] = {0xdd, 0x05, 0x24, 0x00, 0x08};//关风扇
  //<24关警报、关风扇 24<tmp<30开风扇  >30报警且开风扇
  if (tmp <= 20) {
    write_uart_data(fd, alert_off);
    write_uart_data(fd, fan_off);
  } else if (tmp > 20 && tmp < 30) {
    write_uart_data(fd, alert_off);
    write_uart_data(fd, fan_on);
  } else {
    write_uart_data(fd, fan_on);
    write_uart_data(fd, alert_on);
  }
  return fd;
}

//给仿真平台发命令
int write_uart_data(int fd, unsigned char cmd[]) {
  int ret = write(fd, cmd, 5);
  if (ret == -1) {
    perror("写串口数据失败");
    return -1;
  }
  return fd;
}

//------
//读取仿真平台数据
int read_uart_data(int fd, char data[], char coll_data[3]) {
  int ret = read(fd, data, 36);
  if (ret == -1) {
    perror("读串口数据失败！");
    return -1;
  }
  coll_data[0] = data[5];
  coll_data[1] = data[7];
  coll_data[2] = data[20];
  printf("tmp:%d     hum:%d     light:%d\n", coll_data[0], coll_data[1], coll_data[2]);
  return fd;
}

// ---------
//打开串口
int open_uart() {
  int fd;
  //读写方式|不会使打开的文件成为这个进程的控制终端|非阻塞方式
  fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
  if (-1 == fd) {
    perror("串口打开失败!");
    return -1;
  }
  //恢复串口阻塞状态
  if (fcntl(fd, F_SETFL, 0)) {
    perror("fcntl failed!");
  } else {
    printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
  }
  //测试是否是终端设备
  if (isatty(STDIN_FILENO) == 0) {
    perror("标准输入不是终端设备！");
  } else {
    perror("isatty success!");
  }
  return fd;
}
//----
//初始化串口
int init_uart(int fd) {
  struct termios opt;

  if (tcgetattr(fd, &opt) != 0) {
    perror("串口不可用！");
    return -1;
  }

  //波特率115200
  cfsetispeed(&opt, B115200);
  cfsetospeed(&opt, B115200);

  //程序不被占用
  opt.c_cflag |= CLOCAL;

  //串口读取输入数据
  opt.c_cflag |= CREAD;
  opt.c_oflag &= ~(ONLCR | OCRNL);     //修改输出模式
  opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);    //修改输入模式
  opt.c_iflag &= ~(ICRNL | INLCR);    //修改输入模式
  opt.c_iflag &= ~(IXON | IXOFF | IXANY);    //修改输入模式
  opt.c_cflag &= ~CRTSCTS;     //设置数据流控制, 不使用流控制
  opt.c_cflag &= ~CSIZE;     //先清除数据位,再配置
  opt.c_cflag |= CS8;     //设置数据位,CS5,CS6,CS7,CS8
  opt.c_cflag &= ~PARENB;     //禁止使用奇偶校验
  opt.c_iflag &= ~INPCK;     //禁止输入奇偶校验
  opt.c_cflag &= ~CSTOPB;     //设置停止位, 一位停止位
  opt.c_oflag &= ~OPOST;     //修改输出模式，数据不经处理输出, 原始数据输出
  opt.c_lflag &= ~(ICANON | ECHO | ECHOE
      | ISIG);     //ICANON 使用标准输入模式，ECHO回显输入字符，~(ECHOE|ISIG)在进行write写输 出的时候，不必以回车的键入作为输出的结束，直接发送输入值。
  opt.c_cc[VTIME] = 1;     //设置等待时间, 读取一个字符等待0*(0/10)s
  opt.c_cc[VMIN] = 1;     //设置最小接收字符,读取字符的最少个数为0
  tcflush(fd, TCIFLUSH);     //如果发生数据溢出，接收数据，但是不再读取,刷新收到的数据但是不读

  //激活配置
  if (tcsetattr(fd, TCSANOW, &opt) != 0) {
    perror("串口初始化失败！");
    return -1;
  }
  printf("串口初始化成功！\n");

  return 0;
}

