#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <CANopen.h>

// Demux TPDO на общей UART (CANopen libdeps — одна нода на update):
//   TPDO2 0x280+N — ток + момент (0.1% номинала)
//   TPDO3 0x380+N — позиция + скорость

namespace CanTpdo {

struct Node {
    int32_t pos;
    int32_t vel;
    int16_t current;
    int16_t torque;
};

inline Node* nodes() {
    static Node n[128];
    static bool z = false;
    if (!z) {
        memset(n, 0, sizeof(Node) * 128);
        z = true;
    }
    return n;
}

inline int32_t rdI32(const uint8_t* p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

inline int16_t rdI16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

inline uint32_t id11(const uint8_t* f) {
    return (((uint32_t)f[1] << 24) | ((uint32_t)f[2] << 16) |
            ((uint32_t)f[3] << 8) | f[4]) & 0x7FFu;
}

inline void poll(HardwareSerial& serial) {
    while (serial.available() >= 13) {
        uint8_t dlc = (uint8_t)serial.peek() & 0x0F;
        if (dlc < 1 || dlc > 8) {
            serial.read();
            continue;
        }
        uint8_t f[13];
        if (serial.readBytes(f, 13) != 13) break;

        uint32_t id = id11(f);
        uint8_t node = (uint8_t)(id & 0x7Fu);
        if (node == 0) continue;

        Node& n = nodes()[node];
        const uint8_t* d = &f[5];

        if (id == (CAN_TPDO2(node) & 0x7FFu)) {
            if (dlc >= 2) n.current = rdI16(&d[0]);
            if (dlc >= 4) n.torque  = rdI16(&d[2]);
            continue;
        }
        if (id == (CAN_TPDO3(node) & 0x7FFu)) {
            if (dlc >= 4) n.pos = rdI32(&d[0]);
            if (dlc >= 8) n.vel = rdI32(&d[4]);
        }
    }
}

inline int32_t position(uint8_t node) { return nodes()[node].pos; }
inline int32_t velocity(uint8_t node) { return nodes()[node].vel; }
inline int16_t currentRaw(uint8_t node) { return nodes()[node].current; }
inline int16_t torqueRaw(uint8_t node)  { return nodes()[node].torque; }
inline float currentPercent(uint8_t node) { return nodes()[node].current * 0.1f; }
inline float torquePercent(uint8_t node)  { return nodes()[node].torque * 0.1f; }

// TPDO3 vel: ед. команды/с → °/с (CANOPEN_UNITS_PER_REV ед. = 1 оборот)
inline float velocityDegPerSec(uint8_t node) {
    return (float)nodes()[node].vel * 360.0f / (float)CANOPEN_UNITS_PER_REV;
}

// об/мин
inline float velocityRpm(uint8_t node) {
    return (float)nodes()[node].vel * 60.0f / (float)CANOPEN_UNITS_PER_REV;
}

} // namespace CanTpdo
