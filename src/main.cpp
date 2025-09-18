// The Bluetooth MAC address of the ESP32 must be uploaded 
// to the PS4 controller using the Sixaxis pairing tool.
// A0:DD:6C:85:54:9E

#include <Arduino.h>
#include <math.h>
#include <SCServo.h>
#include <PS4Controller.h>
#include "board.h"

SMS_STS st;
SCSCL sc;

#define S_RXD 18
#define S_TXD 19

#define S_SCL 22
#define S_SDA 21

#define RGB_LED   23
#define NUMPIXELS 10

const float L1 = 115.0;  
const float L2 = 115.0;  

const float l1 = L1;
const float l2 = L2;

const int SERVO_LIMITS[5][2] = {
    {0, 0},
    {1024, 3072},
    {0, 2048},
    {400, 3700},
    {600, 3500}
};

struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
};

struct Point3D {
    float x;
    float y;
    float z;
}; 

// Definicje przycisków PS4
#define PS4_TRIANGLE PS4_BUTTON_TRIANGLE
#define PS4_CIRCLE PS4_BUTTON_CIRCLE
#define PS4_CROSS PS4_BUTTON_CROSS

// Fixed variable declarations
Point3D current_position = {150.0, 0.0, 100.0};
float initial_x = 150.0;
float initial_z = 100.0;
String orientation_mode = "flat";
String gripper_state = "close";
float current_x = initial_x;
float current_z = initial_z;
int base_position = 2048; // Center position (0-4095 range)
int base_speed = 500;
const float DEADZONE = 0.1;
const float STEP_SIZE = 2.0;

// Button state tracking
bool last_cross = false;
bool last_circle = false;
bool last_r1 = false;


bool checkServoAngles(int servo_angles[4]) {
    bool all_valid = true;
    
    for (int i = 0; i < 4; i++) {
        int servo_id = i + 1;
        int target = servo_angles[i];
        int min_angle = SERVO_LIMITS[servo_id][0];
        int max_angle = SERVO_LIMITS[servo_id][1];
        
        if (target < min_angle || target > max_angle) {
            Serial.printf("Servo %d angle %d out of range (%d-%d)\n", 
                         servo_id, target, min_angle, max_angle);
            all_valid = false;
        }
    }
    return all_valid;
}

int radToServo(float rad) {
    const int center = 2048;
    const float scale = 2048.0f / M_PI;
    return 4095 - (int)round(rad * scale + center);
}

bool solveIK2D(float x_target, float z_target, String orientation, float &theta2, float &theta3, float &theta4) {
  
    float d = sqrt(x_target * x_target + z_target * z_target);
    
    // Check if target is within reach
    if (d > (l1 + l2)) {
        Serial.println("Target out of reach");
        return false;
    }
    
    // Calculate cos(theta3)
    float cos_theta3 = (x_target * x_target + z_target * z_target - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    
    // Check if solution exists
    if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
        Serial.println("No IK solution exists");
        return false;
    }
    
    // Calculate joint angles
    theta3 = -acos(cos_theta3);
    
    float k1 = l1 + l2 * cos(theta3);
    float k2 = l2 * sin(theta3);
    
    theta2 = atan2(z_target, x_target) - atan2(k2, k1);
    
    // Calculate theta4 based on orientation
    if (orientation == "down") {
        theta4 = (-PI/2) - (theta2 + theta3);
    } else { // flat
        theta4 = 0 - (theta2 + theta3);
    }
    
    return true;
}

void controlGripper(String action) {
  int position;
  if (action == "open") {
    position = 800; 
  } else {
    position = 500; 
  }
  
  sc.WritePos(5, position, 0, 1500); 
}

void moveServosSyncEx(uint8_t ids[4], int targetPos[4], u16 baseSpeed, u16 accValue) {
    s16 currentPos[4];
    u16 speed[4];
    s16 position[4];
    u8 acc[4];

    int delta[4];
    int maxDelta = 0;

    for (int i = 0; i < 4; i++) {
        currentPos[i] = st.ReadPos(ids[i]);
        delta[i] = abs(targetPos[i] - currentPos[i]);
        if (delta[i] > maxDelta) maxDelta = delta[i];
        acc[i] = (u8)accValue;
    }

    for (int i = 0; i < 4; i++) {
        if (delta[i] == 0) {
            speed[i] = 0;
        } else {
            speed[i] = (u16)((float)delta[i] / maxDelta * baseSpeed);
            if (speed[i] < 1) speed[i] = 1;
        }
        position[i] = targetPos[i];
    }

    st.SyncWritePosEx(ids, 4, position, speed, acc);
}

bool moveToPoint2D(float x, float z, String orientation, int base_rot, int max_speed) {
    float theta2, theta3, theta4;
    
    // Solve inverse kinematics
    if (!solveIK2D(x, z, orientation, theta2, theta3, theta4)) {
        return false;
    }
    
    // Convert to servo positions
    int servo_angles[4];
    servo_angles[0] = base_rot;                // Base
    servo_angles[1] = radToServo(theta2);      // Joint 1
    servo_angles[2] = radToServo(theta3);      // Joint 2  
    servo_angles[3] = radToServo(theta4);      // Joint 3
    
    // Check servo angle limits
    if (!checkServoAngles(servo_angles)) {
        return false;
    }
    
    // Prepare servo IDs and positions for sync movement
    uint8_t servo_ids[4] = {1, 2, 3, 4};
    moveServosSyncEx(servo_ids, servo_angles, max_speed, 50);
    
    return true;
}

void process_PS4_input(float delta_time) {
    // Get joystick values (-128 to 127, normalize to -1.0 to 1.0)
    float lx = -PS4.LStickY() / 128.0; // Left stick Y for X movement
    float ry = -PS4.RStickY() / 128.0; // Right stick Y for Z movement  
    float rx = PS4.RStickX() / 128.0; // Right stick X for base rotation
    
    // Apply deadzone
    lx = (abs(lx) < DEADZONE) ? 0 : lx;
    ry = (abs(ry) < DEADZONE) ? 0 : ry;
    rx = (abs(rx) < DEADZONE) ? 0 : rx;
    
    // Apply nonlinear response
    float lx_nonlinear = lx * abs(lx);
    float ry_nonlinear = ry * abs(ry);
    float rx_nonlinear = rx * abs(rx);
    
    // Calculate new position
    float new_x = current_x + (-lx_nonlinear * STEP_SIZE);
    float new_z = current_z - (ry_nonlinear * STEP_SIZE);
    
    bool needs_move = false;

    // Handle base rotation
    if (abs(rx_nonlinear) > DEADZONE) {
        base_position += (int)(rx_nonlinear * base_speed * delta_time);
        base_position = constrain(base_position, 0, 4095);
        needs_move = true;
    }
    
    // Check if position changed
    if ((new_x != current_x || new_z != current_z) || needs_move) {
        if (moveToPoint2D(new_x, new_z, orientation_mode, base_position, 1000)) {
            current_x = new_x;
            current_z = new_z;
            
            Serial.printf("Position: (%.2f, %.2f), Base: %d\n", 
                         current_x, current_z, base_position);
        } else {
            Serial.println("Position unreachable!");
        }
    }
}

void processButtons() {
    // Cross button - down orientation
    if (PS4.Cross() && !last_cross) {
        orientation_mode = "down";
        moveToPoint2D(current_x, current_z, orientation_mode, base_position, 500);
        Serial.println("Orientation: DOWN");
    }
    
    // Circle button - flat orientation  
    if (PS4.Circle() && !last_circle) {
        orientation_mode = "flat";
        moveToPoint2D(current_x, current_z, orientation_mode, base_position, 500);
        Serial.println("Orientation: FLAT");
    }

    // R1 button - toggle gripper
    if (PS4.R1() && !last_r1) {
        if (gripper_state == "close") {
        controlGripper("open");
        gripper_state = "open";
        Serial.println("Gripper: OPEN");
        } else {
        controlGripper("close");
        gripper_state = "close";
        Serial.println("Gripper: CLOSE");
        }
    }
    
    // Update button states
    last_cross = PS4.Triangle();
    last_circle = PS4.Circle();
    last_r1 = PS4.R1();
}

void onConnect() {
    Serial.println("PS4 controller connected");
}

void onDisconnect() {
    Serial.println("PS4 controller disconnected");
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    sc.pSerial = &Serial1;
    
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);
    PS4.begin(); 
    delay(1000);
    
    InitScreen();

    moveToPoint2D(current_x, current_z, orientation_mode, base_position, 500);

    btMac();
    scanServos();
    displayResultsScreen();

    Serial.println("Inicjalizacja zakończona");

    
}

void loop() {
    static unsigned long last_time = 0;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    last_time = current_time;
    
    if (PS4.isConnected()) {
        process_PS4_input(delta_time);
        processButtons();
    }
    
    delay(20); // Small delay for stability
}
