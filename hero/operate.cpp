#include <Arduino.h>
#include "operate.hpp"

OPERATE operate;

void OPERATE::begin(){
  Serial2.begin(100000, SERIAL_8E1);
}

void OPERATE::update(){
  static uint_fast8_t data[18] = { 0 };
  static uint_fast8_t *data_pointer = data;
  if(Serial2.available() == 18 && available_count < 50){
    for(data_pointer = data; data_pointer < data + 18; data_pointer++) *data_pointer = Serial2.read();
    available_count = 50;
  }
  else if(available_count == 50){
    rx = (((data[1] & 0x07) <<  8) |  data[0]                       ) - 1024;
    ry = (((data[2] & 0x3F) <<  5) | (data[1] >> 3)                 ) - 1024;
    lx = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)) - 1024;
    ly = (((data[5] & 0x0F) <<  7) | (data[4] >> 1)                 ) - 1024;
    ls = (data[5] & 0xC0) >> 6;
    rs = (data[5] & 0x30) >> 4;
    sr = ((data[17] << 8) | data[16]) - 1024;
    mx = ((data[7] << 8) | data[6]);
    my = ((data[9] << 8) | data[8]);
    lc = data[12] & 0x01;
    rc = data[13] & 0x01;
    wk =  data[14]       & 0x01;
    sk = (data[14] >> 1) & 0x01;
    ak = (data[14] >> 2) & 0x01;
    dk = (data[14] >> 3) & 0x01;
    sh = (data[14] >> 4) & 0x01;
    ct = (data[14] >> 5) & 0x01;
    qk = (data[14] >> 6) & 0x01;
    ek = (data[14] >> 7) & 0x01;
    rk =  data[15]       & 0x01;
    fk = (data[15] >> 1) & 0x01;
    gk = (data[15] >> 2) & 0x01;
    zk = (data[15] >> 3) & 0x01;
    xk = (data[15] >> 4) & 0x01;
    ck = (data[15] >> 5) & 0x01;
    vk = (data[15] >> 6) & 0x01;
    bk = (data[15] >> 7) & 0x01;
    available_count--;
  }
  else if(Serial2.available() > 18){
    available_count = 0;
    do {
      Serial2.read();
    } while(Serial2.available());
  }
  else if(available_count == 0){
    rx = 0;
    ry = 0;
    lx = 0;
    ly = 0;
    ls = 0;
    rs = 0;
    sr = 0;
    mx = 0;
    my = 0;
    lc = 0;
    rc = 0;
    wk = 0;
    sk = 0;
    ak = 0;
    dk = 0;
    sh = 0;
    ct = 0;
    qk = 0;
    ek = 0;
    rk = 0;
    fk = 0;
    gk = 0;
    zk = 0;
    xk = 0;
    ck = 0;
    vk = 0;
    bk = 0;
  }
  else available_count--;
}
