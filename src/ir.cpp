#include "ir.h"

void send_ir(ir_msg &msg, int kIrLed){
    //msg {[protocol], [power], [temp], [fan_speed], [mode]}
    // kIrLed : IR LED pin

    IRac ac(kIrLed);  // Create a A/C object using GPIO to sending messages with.

    // setting up states of the AC
    // protocol LG2 and KELON
    decode_type_t protocol;
    // if(msg.protocol == 1) {protocol = LG2;}
    // else if (msg.protocol == 2){ protocol = KELON;}
    // else if (msg.protocol == 3){ protocol = COOLIX;}
    // else if (msg.protocol == 4){ protocol = SONY;}
    // else if (msg.protocol == 5){ protocol = DAIKIN;}
    // else if (msg.protocol == 6){ protocol = HAIER_AC;}
    // else if (msg.protocol == 7){ protocol = WHIRLPOOL_AC;}
    // else if (msg.protocol == 8){ protocol = TEKNOPOINT;}
    // else if (msg.protocol == 9){ protocol = GREE;}
    // else if (msg.protocol == 10){ protocol = TCL112AC;}
    // else if (msg.protocol == 11){ protocol = TCL96AC;}
    // else if (msg.protocol == 12){ protocol = SAMSUNG;}
    // else if (msg.protocol == 13){ protocol = PRONTO;}
    // else if (msg.protocol == 14){ protocol = PIONEER;}
    //decode_type_t protocol = (msg.protocol==1) ? LG2:KELON;
    //ac.next.protocol = protocol;

    ac.next.protocol = static_cast<decode_type_t>(msg.protocol);

    // // power on/off
    bool power_state;
    if(msg.power == 1){power_state = true;}
    else if(msg.power == 0){power_state = false;}
    else if(msg.power == 2){power_state = NULL;}  
    ac.next.power = power_state;
    // bool power_state = (msg.power ==1) ?true: false;
    // ac.next.power = power_state;

    // temperature
    ac.next.celsius = true;  // Use Celsius for temp units. False = Fahrenheit
    ac.next.degrees = msg.temp;  // set degrees.

    // fan speed
    // stdAc::fanspeed_t fan_speed;
    // if (msg.fan_speed==0)       fan_speed = stdAc::fanspeed_t::kAuto;
    // else if (msg.fan_speed==1)  fan_speed = stdAc::fanspeed_t::kMin;
    // else if (msg.fan_speed==2)  fan_speed = stdAc::fanspeed_t::kLow;
    // else if (msg.fan_speed==3)  fan_speed = stdAc::fanspeed_t::kMedium;
    // else if (msg.fan_speed==4)  fan_speed = stdAc::fanspeed_t::kHigh;
    // else                        fan_speed = stdAc::fanspeed_t::kMax;
    stdAc::fanspeed_t fan_speed;
    if (msg.fan_speed==0)       fan_speed = stdAc::fanspeed_t::kAuto;
    else if (msg.fan_speed==1)  fan_speed = stdAc::fanspeed_t::kLow;
    else if (msg.fan_speed==2)  fan_speed = stdAc::fanspeed_t::kLow;
    else if (msg.fan_speed==3)  fan_speed = stdAc::fanspeed_t::kMedium;
    else if (msg.fan_speed==4)  fan_speed = stdAc::fanspeed_t::kHigh;
    else                        fan_speed = stdAc::fanspeed_t::kMax;

    ac.next.fanspeed = fan_speed; 

    // mode
    // stdAc::opmode_t mode_ac;
    // if (msg.mode==-1)       mode_ac = stdAc::opmode_t::kOff;
    // else if (msg.mode==0)   mode_ac = stdAc::opmode_t::kAuto;
    // else if (msg.mode==1)   mode_ac = stdAc::opmode_t::kCool;
    // else if (msg.mode==2)   mode_ac = stdAc::opmode_t::kHeat;
    // else if (msg.mode==3)   mode_ac = stdAc::opmode_t::kDry;
    // else                    mode_ac = stdAc::opmode_t::kFan;
    stdAc::opmode_t mode_ac;
    if (msg.mode==-1)       mode_ac = stdAc::opmode_t::kCool;
    else if (msg.mode==0)   mode_ac = stdAc::opmode_t::kCool;
    else if (msg.mode==1)   mode_ac = stdAc::opmode_t::kCool;
    else if (msg.mode==2)   mode_ac = stdAc::opmode_t::kCool;
    else if (msg.mode==3)   mode_ac = stdAc::opmode_t::kCool;
    else                    mode_ac = stdAc::opmode_t::kCool;

    ac.next.mode = mode_ac;  // Run in cool mode initially.

    //Other configurations of the AC
    ac.next.swingv = stdAc::swingv_t::kOff;  // Don't swing the fan up or down.
    ac.next.swingh = stdAc::swingh_t::kOff;  // Don't swing the fan left or right.
    ac.next.light = false;  // Turn off any LED/Lights/Display that we can.
    ac.next.beep = false;  // Turn off any beep from the A/C if we can.
    ac.next.econo = false;  // Turn off any economy modes if we can.
    ac.next.filter = false;  // Turn off any Ion/Mold/Health filters if we can.
    ac.next.turbo = false;  // Don't use any turbo/powerful/etc modes.
    ac.next.quiet = false;  // Don't use any quiet/silent/etc modes.
    ac.next.sleep = -1;  // Don't set any sleep time or modes.
    ac.next.clean = false;  // Turn off any Cleaning options if we can.
    ac.next.clock = -1;  // Don't set any current time if we can avoid it.

    //sending message to the AC

    ac.sendAc();  // Have the IRac class create and send a message.
    delay(50);
    ac.sendAc();  // Have the IRac class create and send a message.
    delay(50);
    ac.sendAc();  // Have the IRac class create and send a message.

    //msg {[protocol], [power], [temp], [fan_speed], [mode]}
    Serial.println("IR message sent");
    Serial.print("protocol:");Serial.println(msg.protocol);
    Serial.print("power:");Serial.println(msg.power);
    Serial.print("temperature:");Serial.println(msg.temp);
    Serial.print("mode:");Serial.println(msg.mode);
    Serial.print("fan speed:");Serial.println(msg.fan_speed);

    delay(100);  
}

// void update_ir(ir_msg &msg){
//     msg.protocol    = protocol;
//     msg.power       = power;
//     msg.fan_speed   = fan_speed;
//     msg.mode        = mode;
//     msg.temp        = temperature;
// }

