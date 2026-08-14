#pragma once

// UART → UART–CAN конвертер (в master: PE0/PE1 = newSerial)
#define PIN_CAN_RX          PE0
#define PIN_CAN_TX          PE1

#define PIN_LED_GREEN       PD15
#define PIN_LED_RED         PD14

HardwareSerial canSerial(PIN_CAN_RX, PIN_CAN_TX);