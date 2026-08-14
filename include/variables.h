#pragma once


enum SYSTEM_STATES : uint8_t {
    SS_MAIN_MENU,
    SS_MOVABLE_SURFACE_WALK,
};

SYSTEM_STATES system_state = SS_MAIN_MENU;
uint8_t system_state_last = 0xFF;

bool change_leg = false;
uint8_t period_seconds = 12;
int target_cycles = 5;
float scale_x_for_trajectory = 1.0f;
float scale_z_for_trajectory = 1.0f;

bool isSSChange() {
    bool changed = (system_state != system_state_last);
    system_state_last = system_state;
    return changed;
}
