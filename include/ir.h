#ifndef IR_H

#define IR_H

#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRutils.h>
// #include "MQTT.h"
// #include "esp32-mqtt.h"

// extern char* protocol;
// extern int power;
// extern int temperature;
// extern int mode;
// extern int fan_speed;

struct ir_msg{
  int protocol;
  int power;
  int temp;
  int fan_speed;
  int mode;
};

extern void send_ir(ir_msg &msg, int kIrLed);
  
extern void update_ir(ir_msg &msg); 

#endif