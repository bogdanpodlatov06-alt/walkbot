#pragma once

// Node-ID (подставьте свои из Motion Studio)
static const uint8_t NID_HIP  = 1;
static const uint8_t NID_KNEE = 2;
static const uint8_t NID_FOOT = 3;

CANopen hip  (canSerial, NID_HIP);
CANopen knee (canSerial, NID_KNEE);
CANopen foot (canSerial, NID_FOOT);

int32_t cmdTarget[128];

inline void updateAllDrives() {
    CanTpdo::poll(canSerial);
}

inline void initAllDrives() {
    hip.initDrive(20, 2000, 2000);
    knee.initDrive(20, 2000, 2000);
    foot.initDrive(20, 2000, 2000);
}

inline void go(CANopen& drive, uint8_t node, int32_t pos) {
    cmdTarget[node] = pos;
    drive.streamAbsolute(pos);
}

inline void stopAxis(CANopen& drive) {
    drive.halt(true);
}

inline bool arrived(uint8_t node, int32_t window = 100) {
    int32_t err = CanTpdo::position(node) - cmdTarget[node];
    if (err < 0) err = -err;
    int32_t vel = CanTpdo::velocity(node);
    if (vel < 0) vel = -vel;
    return (err <= window) && (vel <= 200);
}

inline bool legArrived() {
    return arrived(NID_HIP) && arrived(NID_KNEE) && arrived(NID_FOOT);
}

struct limbSegment {
    float length;
    float baseAngle;
    float zeroLength;
    float L1;
    float L2;
};

limbSegment segHip  = { 0.42f, 148.22f, 275.07f, 150.97f, 135.0f };
limbSegment segKnee = { 0.45f, 125.0f,  238.08f, 183.0f,   80.0f };
limbSegment segFoot = { 0.0f,   90.0f,  199.72f, 183.0f,   80.0f };

const float ORIGINAL_FOURIER_CYCLE_TIME = 1.134f;

float l1_perfect = 0.42f, l2_perfect = 0.45f, z_offset_perfect = 0.095f;
float z_offset = (segHip.length - l1_perfect) + (segKnee.length - l2_perfect) + z_offset_perfect;

float A0X = 0.0f, A0Z = 0.0f;
float A1X = 0.0f, A1Z = 0.0f;
float A2X = 0.0f, A2Z = 0.0f;
float q1_des = 0.0f, q21_des = 0.0f;
float x_des = 0.0f, z_des = 0.0f;
float q3_des = 0.0f;

const long STEPS_PER_REV = 10000;
const float SCREW_PITCH = 5.0f;
const float MM_PER_STEP = SCREW_PITCH / STEPS_PER_REV;
unsigned long trajectoryStartTime = 0;

inline float angleToLength(float target_angle, const limbSegment& segment) {
    float buf_rad = radians(segment.baseAngle - target_angle);
    return sqrt(segment.L1 * segment.L1 + segment.L2 * segment.L2 - 2 * segment.L1 * segment.L2 * cosf(buf_rad));
}

inline long angleToSteps(float target_angle, const limbSegment& segment) {
    float target_length = angleToLength(target_angle, segment);
    float length_offset = segment.zeroLength - target_length;
    return (long)(length_offset / MM_PER_STEP);
}

inline float stepsToLength(long target_steps, const limbSegment& segment) {
    return segment.zeroLength - (target_steps * MM_PER_STEP);
}

inline float stepsToAngle(long target_steps, const limbSegment& segment) {
    float current_length = stepsToLength(target_steps, segment);
    float buf_rad = acosf((current_length * current_length - segment.L1 * segment.L1 - segment.L2 * segment.L2) / (-2.0f * segment.L1 * segment.L2));
    return segment.baseAngle - degrees(buf_rad);
}

inline void directKinematics(float q_hip, float q_knee) {
    A0X = 0.0f;
    A0Z = 0.0f;
    A1X = A0X + segHip.length * sinf(q_hip);
    A1Z = A0Z - segHip.length * cosf(q_hip);
    A2X = A1X + segKnee.length * sinf(q_hip + q_knee);
    A2Z = A1Z - segKnee.length * cosf(q_hip + q_knee);
}

inline void inverseKinematics(float x_set, float z_set, float rq1, float rq21,
                              float x_start, float z_start) {
    q1_des = -(segKnee.length + z_set * cosf(rq1 + rq21) - z_start * cosf(rq1 + rq21)
               - x_set * sinf(rq1 + rq21) + x_start * sinf(rq1 + rq21)
               + segHip.length * cosf(rq21) - segHip.length * rq1 * sinf(rq21))
             / (segHip.length * sinf(rq21));
    q21_des = rq21
              + ((segKnee.length * cosf(rq1 + rq21) + segHip.length * cosf(rq1))
                 * (z_set - z_start + segKnee.length * cosf(rq1 + rq21) + segHip.length * cosf(rq1)))
                / (segHip.length * segKnee.length * sinf(rq21))
              + ((segKnee.length * sinf(rq1 + rq21) + segHip.length * sinf(rq1))
                 * (x_start - x_set + segKnee.length * sinf(rq1 + rq21) + segHip.length * sinf(rq1)))
                / (segHip.length * segKnee.length * sinf(rq21));

    q1_des  = constrain(q1_des,  -0.261799f,  1.48353f);
    q21_des = constrain(q21_des, -1.5708f,   -0.0523599f);
}

inline void fourierTrajectoryFoot(float x, float scale_x, float scale_z) {
    static const struct {
        const float a0 = -0.01146f;
        const float a1 = -0.2165f, b1 = 0.06321f;
        const float a2 = -0.02297f, b2 = -0.04377f;
        const float a3 = 0.007099f, b3 = 0.001872f;
        const float w = 5.55f;
    } xp;
    static const struct {
        const float a0 = -0.7137f;
        const float a1 = 0.0309f, b1 = 0.02791f;
        const float a2 = 0.00792f, b2 = 0.02081f;
        const float a3 = -0.001175f, b3 = 0.01145f;
        const float a4 = -0.004058f, b4 = 0.002241f;
        const float w = 5.55f;
    } zp;

    float wx = x * xp.w;
    x_des = xp.a0 + scale_x * (xp.a1 * cosf(wx) + xp.b1 * sinf(wx)
            + xp.a2 * cosf(2 * wx) + xp.b2 * sinf(2 * wx)
            + xp.a3 * cosf(3 * wx) + xp.b3 * sinf(3 * wx));

    float wz = x * zp.w;
    z_des = zp.a0 + scale_z * (zp.a1 * cosf(wz) + zp.b1 * sinf(wz)
            + zp.a2 * cosf(2 * wz) + zp.b2 * sinf(2 * wz)
            + zp.a3 * cosf(3 * wz) + zp.b3 * sinf(3 * wz)
            + zp.a4 * cosf(4 * wz) + zp.b4 * sinf(4 * wz));
}

inline void fourierTrajectoryAnkle(float x) {
    static const struct {
        const float a0 = -2.764f;
        const float a1 = -1.038f, b1 = -4.669f;
        const float a2 = -2.679f, b2 = -3.29f;
        const float a3 = -0.5532f, b3 = -3.817f;
        const float w = 5.55f;
    } zp;

    float wx = x * zp.w;
    q3_des = zp.a0 + zp.a1 * cosf(wx) + zp.b1 * sinf(wx)
             + zp.a2 * cosf(2.0f * wx) + zp.b2 * sinf(2.0f * wx)
             + zp.a3 * cosf(3.0f * wx) + zp.b3 * sinf(3.0f * wx);
}


inline void setStopMotors() {
    stopAxis(hip);
    stopAxis(knee);
    stopAxis(foot);
}

void setPressetSettings() {
    hip.setSpeed(20);
    knee.setSpeed(20);
    foot.setSpeed(20);

    hip.setAcceleration(3000);
    knee.setAcceleration(3000);
    foot.setAcceleration(3000);

    hip.setDeceleration(4000);
    knee.setDeceleration(3000);
    foot.setDeceleration(3000);
}

struct MotorPreset {
    float speed;
    float acceleration;
    float deceleration;
};

struct TrajectoryPreset {
    int time;
    MotorPreset motors[3];
};

const TrajectoryPreset TRAJECTORY_PRESETS[] = {
    {2,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {3,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {4,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {5,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {6,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {7,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {8,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {9,  {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {10, {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {11, {{3000, 200, 500}, {3000, 200, 500}, {3000, 200, 500}}},
    {12, {{2000, 200, 2000}, {2000, 200, 1800}, {2000, 300, 2000}}},
    {13, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {14, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {15, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {16, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {17, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {18, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {19, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
    {20, {{2000, 200, 2000}, {2000, 250, 1800}, {2000, 300, 2000}}},
};

const int NUM_PRESETS = 19;
MotorPreset motorParams[3];

bool getMotorParamsForTime(int desiredTime, MotorPreset result[3]) {
    for (int i = 0; i < NUM_PRESETS; i++) {
        if (TRAJECTORY_PRESETS[i].time == desiredTime) {
            for (int m = 0; m < 3; m++) result[m] = TRAJECTORY_PRESETS[i].motors[m];
            return true;
        }
    }
    return false;
}

void setMotor(int motorIndex, float speed, float accel, float decel) {
    CANopen* d = nullptr;
    if (motorIndex == 0) d = &hip;
    else if (motorIndex == 1) d = &knee;
    else if (motorIndex == 2) d = &foot;
    if (!d) return;
    d->setSpeed((uint32_t)speed);
    d->setAcceleration((uint32_t)accel);
    d->setDeceleration((uint32_t)decel);
}

struct ParsedData {
    String command;
    float value[14];
    int valueCount;
};

ParsedData parsedInput(String input) {
    ParsedData result;
    result.valueCount = 0;
    int separatorPosition[10];
    int separatorCount = 0;

    for (int i = 0; i < input.length(); i++) {
        if (input[i] == ';' || input[i] == ':') {
            separatorPosition[separatorCount++] = i;
        }
    }
    if (separatorCount == 0) {
        result.command = input;
        return result;
    }
    result.command = input.substring(0, separatorPosition[0]);
    for (int i = 0; i < separatorCount; i++) {
        int start = separatorPosition[i] + 1;
        int end = (i < separatorCount - 1) ? separatorPosition[i + 1] : input.length();
        result.value[result.valueCount++] = input.substring(start, end).toFloat();
    }
    return result;
}

inline void handleSerialCommand() {
    while (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        ParsedData data = parsedInput(input);

        if (input.equalsIgnoreCase("Stop")) {
            setStopMotors();
            system_state = SS_MAIN_MENU;
            Serial.println("OK:Stop");
            Serial.flush();
            continue;
        }
        if (input.equalsIgnoreCase("Start")) {
            Serial.println("Start");
            Serial.flush();
            system_state = SS_MAIN_MENU;
            continue;
        }
        if (input.equalsIgnoreCase("Error")) {
            byte ok = 0;
            for (int i = 0; i < 3; i++) {
                if (!hip.isFault() && !knee.isFault() && !foot.isFault()) ok++;
            }
            Serial.println(ok >= 2 ? "No" : "Yes");
            Serial.flush();
            continue;
        }
        if (input.equalsIgnoreCase("Ping")) {
            Serial.println("Pong");
            Serial.flush();
            continue;
        }

        if (data.command.equalsIgnoreCase("Walk") && data.valueCount >= 4) {
            if (data.valueCount >= 9) {
                scale_x_for_trajectory = data.value[3] / 10.0f;
                scale_z_for_trajectory = data.value[4] / 10.0f;
                change_leg = data.value[5] != 0.0f;
                period_seconds = (uint8_t)constrain(data.value[6], 2, 20);
                target_cycles = (int)data.value[7];
            } else {
                scale_x_for_trajectory = data.value[0] / 10.0f;
                scale_z_for_trajectory = data.value[1] / 10.0f;
                change_leg = data.valueCount > 2 && data.value[2] != 0.0f;
                period_seconds = (uint8_t)constrain(
                    data.valueCount > 3 ? data.value[3] : 12, 2, 20);
                target_cycles = data.valueCount > 4 ? (int)data.value[4] : 5;
            }
            system_state = SS_MOVABLE_SURFACE_WALK;
            continue;
        }
    }
}

inline void mainControl() {
    handleSerialCommand();
    updateAllDrives();

    switch (system_state) {
        case SS_MAIN_MENU: {
            if (isSSChange()) setPressetSettings();
        } break;

        case SS_MOVABLE_SURFACE_WALK: {
            static int mode = 0;
            static unsigned long lastTelemMs = 0;
            const int INTERPOLATION_STEPS = 300;

            if (isSSChange()) {
                mode = 0;
                trajectoryStartTime = 0;
                lastTelemMs = 0;
                setPressetSettings();
                z_offset = (segHip.length - l1_perfect) + (segKnee.length - l2_perfect) + z_offset_perfect;
            }

            switch (mode) {
                case 0: {
                    float current_q1  = radians(0.1f);
                    float current_q21 = -1 * radians(0.1f);

                    directKinematics(current_q1, current_q21);
                    fourierTrajectoryFoot(0.0f, scale_x_for_trajectory, scale_z_for_trajectory);
                    fourierTrajectoryAnkle(0.0f);

                    float step_X = (x_des - A2X) / INTERPOLATION_STEPS;
                    float step_Z = ((z_des - z_offset) - A2Z) / INTERPOLATION_STEPS;

                    for (int i = 0; i <= INTERPOLATION_STEPS; i++) {
                        float ix = A2X + i * step_X;
                        float iz = A2Z + i * step_Z;
                        inverseKinematics(ix, iz, current_q1, current_q21, A0X, A0Z);
                        current_q1  = q1_des;
                        current_q21 = q21_des;
                    }

                    go(hip,  NID_HIP,  angleToSteps(degrees(current_q1), segHip));
                    go(knee, NID_KNEE, angleToSteps(degrees(-1 * current_q21), segKnee));
                    go(foot, NID_FOOT, angleToSteps(-1 * q3_des, segFoot));

                    // if (legArrived) {
                        if (getMotorParamsForTime(period_seconds, motorParams)) {
                            for (int i = 0; i < 3; i++) {
                                setMotor(i, motorParams[i].speed, motorParams[i].acceleration, motorParams[i].deceleration);
                            }
                        }
                        Serial.print("<ANGLES:");
                        Serial.print((int)degrees(current_q1));
                        Serial.print(",");
                        Serial.print((int)degrees(-1 * current_q21));
                        Serial.print(",");
                        Serial.print((int)(-1 * q3_des));
                        Serial.println(">");
                        Serial.flush();
                        trajectoryStartTime = millis();
                        lastTelemMs = 0;
                        mode = 1;
                    // }
                } break;

                case 1: {
                    unsigned long currentTime = millis();
                    float elapsed_time = (currentTime - trajectoryStartTime) / 1000.0f;
                    float cycle_time = fmod(elapsed_time, (float)period_seconds);

                    float normalized_time = cycle_time / (float)period_seconds;
                    float fourier_input = normalized_time * ORIGINAL_FOURIER_CYCLE_TIME;

                    fourierTrajectoryFoot(fourier_input, scale_x_for_trajectory, scale_z_for_trajectory);
                    fourierTrajectoryAnkle(fourier_input);
                    inverseKinematics(x_des, z_des - z_offset, q1_des, q21_des, A0X, A0Z);

                    go(hip,  NID_HIP,  angleToSteps(degrees(q1_des), segHip));
                    go(knee, NID_KNEE, angleToSteps(degrees(-1 * q21_des), segKnee));
                    go(foot, NID_FOOT, angleToSteps(-1 * q3_des, segFoot));

                    if ((int)(elapsed_time / period_seconds) >= target_cycles) {
                        setPressetSettings();
                        go(hip,  NID_HIP,  angleToSteps(0.0f, segHip));
                        go(knee, NID_KNEE, angleToSteps(0.0f, segKnee));
                        go(foot, NID_FOOT, angleToSteps(0.0f, segFoot));
                        mode = 2;
                    }

                    if (currentTime - lastTelemMs >= 50) {
                        const float dt = (lastTelemMs == 0)
                            ? 0.05f
                            : (currentTime - lastTelemMs) / 1000.0f;
                        lastTelemMs = currentTime;

                        float aHip  = degrees(q1_des);
                        float aKnee = degrees(-1 * q21_des);
                        float aFoot = -1 * q3_des;
                        if (isnan(aHip))  aHip  = 0;
                        if (isnan(aKnee)) aKnee = 0;
                        if (isnan(aFoot)) aFoot = 0;

                        // Угловая скорость команды, °/с
                        static float prevHip = 0, prevKnee = 0, prevFoot = 0;
                        static bool hasPrev = false;
                        float vHip = 0, vKnee = 0, vFoot = 0;
                        if (hasPrev && dt > 1e-4f) {
                            vHip  = (aHip  - prevHip)  / dt;
                            vKnee = (aKnee - prevKnee) / dt;
                            vFoot = (aFoot - prevFoot) / dt;
                        }
                        prevHip = aHip; prevKnee = aKnee; prevFoot = aFoot;
                        hasPrev = true;

                        // Фактическая скорость приводов из TPDO3, °/с
                        float mHip  = CanTpdo::velocityDegPerSec(NID_HIP);
                        float mKnee = CanTpdo::velocityDegPerSec(NID_KNEE);
                        float mFoot = CanTpdo::velocityDegPerSec(NID_FOOT);

                        Serial.print("<ANGLES:");
                        Serial.print(aHip, 2);
                        Serial.print(",");
                        Serial.print(aKnee, 2);
                        Serial.print(",");
                        Serial.print(aFoot, 2);
                        Serial.println(">");

                        // <VEL:cmdH,cmdK,cmdF,actH,actK,actF>  — °/с
                        Serial.print("<VEL:");
                        Serial.print(vHip, 2);
                        Serial.print(",");
                        Serial.print(vKnee, 2);
                        Serial.print(",");
                        Serial.print(vFoot, 2);
                        Serial.print(",");
                        Serial.print(mHip, 2);
                        Serial.print(",");
                        Serial.print(mKnee, 2);
                        Serial.print(",");
                        Serial.print(mFoot, 2);
                        Serial.println(">");

                        Serial.print("<LOAD:");
                        Serial.print(CanTpdo::currentPercent(NID_HIP), 1);
                        Serial.print(",");
                        Serial.print(CanTpdo::currentPercent(NID_KNEE), 1);
                        Serial.print(",");
                        Serial.print(CanTpdo::currentPercent(NID_FOOT), 1);
                        Serial.print(",");
                        Serial.print(CanTpdo::torquePercent(NID_HIP), 1);
                        Serial.print(",");
                        Serial.print(CanTpdo::torquePercent(NID_KNEE), 1);
                        Serial.print(",");
                        Serial.print(CanTpdo::torquePercent(NID_FOOT), 1);
                        Serial.println(">");
                    }
                } break;

                case 2: {
                    // if (legArrived()) {
                        Serial.println("<END>");
                        Serial.flush();
                        system_state = SS_MAIN_MENU;
                    // }
                } break;
            }
        } break;
    }
}
