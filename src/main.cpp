#include <Arduino.h>
#include <HardwareSerial.h>
#include <DButton.h>
#include <CANopen.h>

#include <pinmap.h>
#include <can_tpdo.h>
#include <variables.h>
#include <controlling.h>
#include <communication.h>

#define NEW_ZERO_HIP   0
#define NEW_ZERO_KNEE  0
#define NEW_ZERO_FOOT  0

#define PRINT_POSITION_ENCODER  0

static void zeroAxis(CANopen& axis) {
  axis.setEncoderMode(ENC_CLEAR_MT_ALARM);
  axis.enableServo(true);
  axis.setEncoderMode(ENC_ABS_MULTITURN_LINEAR);
  axis.setZero();
  axis.setEncoderMode(ENC_CLEAR_MT_ALARM);
  axis.setEncoderMode(ENC_ABS_MULTITURN_LINEAR);
  axis.saveSettings();
}

void setup() {
  Serial.begin(115200);

  if (PRINT_POSITION_ENCODER == 0) {
    while (getPermissionToStart());
    while (setPermissionToStart());
  }

  canSerial.begin(115200);
  hip.begin(115200);
  knee.begin(115200);
  foot.begin(115200);

  initAllDrives();

  if (NEW_ZERO_HIP)  zeroAxis(hip);
  if (NEW_ZERO_KNEE) zeroAxis(knee);
  if (NEW_ZERO_FOOT) zeroAxis(foot);
}

void loop() {
  mainControl();

  if (PRINT_POSITION_ENCODER == 1) {
    CanTpdo::poll(canSerial);
    Serial.print("pos H/K/F: ");
    Serial.print(CanTpdo::position(NID_HIP)); Serial.print(" ");
    Serial.print(CanTpdo::position(NID_KNEE)); Serial.print(" ");
    Serial.print(CanTpdo::position(NID_FOOT));
    Serial.println();
  }
}
