#pragma once

inline bool getPermissionToStart() {
    if (Serial.available() > 0) {
        String _input = Serial.readStringUntil('\n');
        _input.trim();
        if (_input.equalsIgnoreCase("Start")) {
            Serial.println("Start");
            Serial.flush();
            return false;
        }
    }
    return true;
}

inline bool setPermissionToStart() {
    if (Serial.available() > 0) {
        String _input = Serial.readStringUntil('\n');
        _input.trim();
        if (_input.equalsIgnoreCase("Error")) {
            Serial.println("Yes");
            Serial.flush();
            return false;
        }
    }
    return true;
}