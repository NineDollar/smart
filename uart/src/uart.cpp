//
// Created by root on 22-9-22.
//
#include "uart.h"
#include "iostream"
#include <stdio.h> /*��׼�����������*/
#include <stdlib.h> /*��׼�����ⶨ��*/
#include <unistd.h> /*Unix ��׼��������*/
#include <string.h>
#include <fcntl.h> /*�ļ����ƶ���*/
#include <termios.h> /*PPSIX �ն˿��ƶ���*/
#include <errno.h> /*����Ŷ���*/
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
  char data[36] = {0};//��������
  char coll_data[3] = {0};//�¶� ʪ�� ����

  int fd = open_uart();
  init_uart(fd);
  while (1) {
    sleep(1);
    //���������ݣ��¶� ʪ�� ���Ȱ�˳����coll_data
    // read_uart_data(fd, data, coll_data);
    /*if(door(fd,1,0)){
        led(fd,coll_data[2],0);
    }*/
    //�Ž�
//	door(fd,1,1);
    //ʪ��
    //humidity(fd,coll_data[1]);
    //·�ƿ���
    //led(fd,coll_data[2],0);
    //�¶ȿ���
    temperature(fd, coll_data[0]);
  }
  return 0;
}

//ʪ�ȿ���
int humidity(int fd, int hum) {

  unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//����
  unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//�ر���
  unsigned char fan_on[] = {0xdd, 0x05, 0x24, 0x00, 0x04};//������
  unsigned char fan_off[] = {0xdd, 0x05, 0x24, 0x00, 0x08};//�ط���
  //<35�ؾ������ط��� 35<hum<60������  >30�����ҿ�����
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

//�Ž�,pic���ͼƬ��Ϣ,close��ſͻ��˹���ָ��
int door(int fd, int pic, int close) {
  unsigned char door_on[] = {0xdd, 0x08, 0x24, 0x00, 0x09};//����
  unsigned char door_off[] = {0xdd, 0x08, 0x24, 0x00, 0x0a};//����
  if (pic) {
    //��Ȩ�޵���������
    if (pic == 1) {
      write_uart_data(fd, door_on);
      return 1;
    } else if (pic == 0) {    //û��Ȩ����������
      write_uart_data(fd, door_off);
      printf("��û��Ȩ��");
      return 0;
    }
  } else if (close) {//���͹�������͹���
    write_uart_data(fd, door_off);
  } else {
    write_uart_data(fd, door_on);
  }
  return 0;
}

// ------
//�ƹ����,pic�������ʶ����Ϣ
//�����ֶ�����,open close���տͻ��˵�ָ��
int led(int fd, int light, int pic) {
  unsigned char led_on[] = {0Xdd, 0X05, 0X24, 0X00, 0X00};//����
  unsigned char led_off[] = {0xdd, 0x05, 0x24, 0x00, 0x01};//�ص�
  unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//����
  unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//�ر���
  //���������ֶ�����
  if (pic) {
    /*if(pic==1){
        //��Ȩ�޵��������ֶ�����
        if(open){
            write_uart_data(fd,led_on);
        }else if(close){
        write_uart_data(fd,led_off);
        }
    }else if(pic==0){
        //��Ȩ��������������һ��ʱ��ر�
        write_uart_data(fd,alert_on);
        sleep(1000);
        write_uart_data(fd,alert_off);
    }*/
  } else {

    //�Զ����صƣ�light<40���ƣ�light>80�ص�
    if (light < 75) {
      write_uart_data(fd, led_on);
    } else if (light >= 75) {
      write_uart_data(fd, led_off);
    }

  }
  return fd;
}

//�¶ȿ���
int temperature(int fd, int tmp) {
  unsigned char alert_on[] = {0xdd, 0x05, 0x24, 0x00, 0x02};//����
  unsigned char alert_off[] = {0xdd, 0x05, 0x24, 0x00, 0x03};//�ر���
  unsigned char fan_on[] = {0xdd, 0x05, 0x24, 0x00, 0x04};//������
  unsigned char fan_off[] = {0xdd, 0x05, 0x24, 0x00, 0x08};//�ط���
  //<24�ؾ������ط��� 24<tmp<30������  >30�����ҿ�����
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

//������ƽ̨������
int write_uart_data(int fd, unsigned char cmd[]) {
  int ret = write(fd, cmd, 5);
  if (ret == -1) {
    perror("д��������ʧ��");
    return -1;
  }
  return fd;
}

//------
//��ȡ����ƽ̨����
int read_uart_data(int fd, char data[], char coll_data[3]) {
  int ret = read(fd, data, 36);
  if (ret == -1) {
    perror("����������ʧ�ܣ�");
    return -1;
  }
  coll_data[0] = data[5];
  coll_data[1] = data[7];
  coll_data[2] = data[20];
  printf("tmp:%d     hum:%d     light:%d\n", coll_data[0], coll_data[1], coll_data[2]);
  return fd;
}

// ---------
//�򿪴���
int open_uart() {
  int fd;
  //��д��ʽ|����ʹ�򿪵��ļ���Ϊ������̵Ŀ����ն�|��������ʽ
  fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
  if (-1 == fd) {
    perror("���ڴ�ʧ��!");
    return -1;
  }
  //�ָ���������״̬
  if (fcntl(fd, F_SETFL, 0)) {
    perror("fcntl failed!");
  } else {
    printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
  }
  //�����Ƿ����ն��豸
  if (isatty(STDIN_FILENO) == 0) {
    perror("��׼���벻���ն��豸��");
  } else {
    perror("isatty success!");
  }
  return fd;
}
//----
//��ʼ������
int init_uart(int fd) {
  struct termios opt;

  if (tcgetattr(fd, &opt) != 0) {
    perror("���ڲ����ã�");
    return -1;
  }

  //������115200
  cfsetispeed(&opt, B115200);
  cfsetospeed(&opt, B115200);

  //���򲻱�ռ��
  opt.c_cflag |= CLOCAL;

  //���ڶ�ȡ��������
  opt.c_cflag |= CREAD;
  opt.c_oflag &= ~(ONLCR | OCRNL);     //�޸����ģʽ
  opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);    //�޸�����ģʽ
  opt.c_iflag &= ~(ICRNL | INLCR);    //�޸�����ģʽ
  opt.c_iflag &= ~(IXON | IXOFF | IXANY);    //�޸�����ģʽ
  opt.c_cflag &= ~CRTSCTS;     //��������������, ��ʹ��������
  opt.c_cflag &= ~CSIZE;     //���������λ,������
  opt.c_cflag |= CS8;     //��������λ,CS5,CS6,CS7,CS8
  opt.c_cflag &= ~PARENB;     //��ֹʹ����żУ��
  opt.c_iflag &= ~INPCK;     //��ֹ������żУ��
  opt.c_cflag &= ~CSTOPB;     //����ֹͣλ, һλֹͣλ
  opt.c_oflag &= ~OPOST;     //�޸����ģʽ�����ݲ����������, ԭʼ�������
  opt.c_lflag &= ~(ICANON | ECHO | ECHOE
      | ISIG);     //ICANON ʹ�ñ�׼����ģʽ��ECHO���������ַ���~(ECHOE|ISIG)�ڽ���writeд�� ����ʱ�򣬲����Իس��ļ�����Ϊ����Ľ�����ֱ�ӷ�������ֵ��
  opt.c_cc[VTIME] = 1;     //���õȴ�ʱ��, ��ȡһ���ַ��ȴ�0*(0/10)s
  opt.c_cc[VMIN] = 1;     //������С�����ַ�,��ȡ�ַ������ٸ���Ϊ0
  tcflush(fd, TCIFLUSH);     //�����������������������ݣ����ǲ��ٶ�ȡ,ˢ���յ������ݵ��ǲ���

  //��������
  if (tcsetattr(fd, TCSANOW, &opt) != 0) {
    perror("���ڳ�ʼ��ʧ�ܣ�");
    return -1;
  }
  printf("���ڳ�ʼ���ɹ���\n");

  return 0;
}

