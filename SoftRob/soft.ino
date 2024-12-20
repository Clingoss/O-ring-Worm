#include "Arduino.h"
#include <Servo.h>

#define PRESSURE_SENSOR (A1)

int E1 = 3;       // Pump 1 Speed
int M1 = 4;       // Pump 1 Direction
int E2 = 11;      // Valve 1 Enable
int M2 = 12;      // Valve 1 State
//const int E3 = 5; // Pump 2 Speed
//const int M3 = 8; // Pump 2 Direction
//const int E4 = 6; // Valve 2 Enable
const int M4 = 7; // Valve 1 Direction


unsigned long lastGaitChange = 0;  // Tracks the last time the state switched
unsigned long gaitDelay = 450;    // Time between state changes (in milliseconds)
int gaitState = 0;                // 0 = Inflate, 1 = Deflate


bool inflateState = false;

Servo servo1;
Servo servo2;
int timecounter = 1;
bool lock = false;
int stateprocess = 0;

const float SensorOffset = 4.44;
const float SensorGain = 0.109;
float pressure = 0;
float pressure_f = 0;
float pressure_a = 0;
float alpha = 0.2;

bool gaitActive = false;

void setup()
{
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    //pinMode(M3, OUTPUT);
    //pinMode(M4, OUTPUT);
    pinMode(PRESSURE_SENSOR, INPUT);

    Serial.begin(115200);
    servo1.attach(9);
    //servo1.
    servo1.write(0);

    servo2.attach(10);
    servo2.write(0);

    Serial.println("Use keys to control:");
    Serial.println("w - Inflate");
    Serial.println("s - Deflate");
    Serial.println("a - Set servo to 0 degrees");
    Serial.println("d - Set servo to 180 degrees");
    Serial.print("Process_Status,");
    Serial.print("Pressure_sensor_Value,");
    Serial.print("Filtered_Pressure,");
    Serial.println(",");
}

void loop()
{
    if (Serial.available() > 0)
    {
        char command = Serial.read();

        switch (command)
        {
        case 'w': // Inflate
            Serial.println("Inflating...");
            inflate(250);
            inflateState = true;
            gaitActive = false;
            break;

        case 'g': //gait
            gaitActive = true;
            break;

        case 's': // Deflate
            deflate();
            Serial.println("Deflating...");
            inflateState = false;
            gaitActive = false;
            break;

        case 'a': // Set servo to 0 degrees, left
            left();
            Serial.println("Servos set to 0 degrees.");
            break;
        case 'd': // Set servo to 180 degrees
            right();
            Serial.println("Servos set to 180 degrees.");
            break;
        case 'x': //steer straight
            straight();
            Serial.println("Servos set to 90 degrees.");
            break;
        default:
            Serial.println("Invalid command.");
            break;
        }
    }

    timecounter++;
    float Setpoint = 20;
    float pressure_sensorValue = (analogRead(PRESSURE_SENSOR) * SensorGain - SensorOffset);
    pressure = pressure_sensorValue;
    pressure_f = pressure_f + alpha * (pressure - pressure_a);
    pressure_a = pressure_f;
    if(pressure_a > 35){
      motor_1_off();
      //delay(500);
      //deflate();
    }
    if(inflateState && pressure_a < 8){
      motor_1_on(250);
    }

    if (gaitActive) {
        // Check the pressure sensor
        float pressure_sensorValue = (analogRead(PRESSURE_SENSOR) * SensorGain - SensorOffset);
        pressure = pressure_sensorValue;
        pressure_f = pressure_f + alpha * (pressure - pressure_a);
        pressure_a = pressure_f;

        // Switch states based on the timing and pressure sensor readings
        if (millis() - lastGaitChange > gaitDelay) {
            lastGaitChange = millis();

            if (gaitState == 0) { // Inflate
                if (pressure_a < 28) { // Ensure pressure is below the threshold
                    inflate(250);
                    gaitState = 1; // Switch to deflate next
                    Serial.println("Inflating...");
                } else {
                    Serial.println("Pressure too high to inflate!");
                }
            } else if (gaitState == 1) { // Deflate
                deflate();
                gaitState = 0; // Switch to inflate next
                Serial.println("Deflating...");
            }
        }
    }




    // if(gaitActive){
    //   inflate(250);
    //   millis(30);
    //   deflate();
    //   millis(30)
    // }

    // Print parameters of the system
    Serial.print(stateprocess);
    Serial.print(",");
    Serial.print(pressure_sensorValue);
    Serial.print(",");
    Serial.print(pressure_f);
    Serial.println(",");

    delay(100);
}

// Motor control functions
void motor_1_on(int motorspeed)
{
    analogWrite(E1, motorspeed);
    digitalWrite(M1, HIGH);
}

void motor_1_off()
{
    analogWrite(E1, 0);
    digitalWrite(M1, LOW);
}

void valve_1_on()
{
    analogWrite(E2, 0);
    digitalWrite(M2, HIGH);
}

void valve_1_off()
{
    analogWrite(E2, 255);
    digitalWrite(M2, HIGH);
}

// Inflate and Deflate Functions
void inflate(int speed)
{
    motor_1_on(speed);
    valve_1_on();
    stateprocess = 1;
}

void deflate()
{
    motor_1_off();
    valve_1_off();
    stateprocess = 2;
}


void left()
{
    int startPos11 = servo1.read(); // Get the current position of servo1
    for (int pos = startPos11; pos <= 180; pos += 1) { // Move from current position to 180 degrees
        servo1.write(pos);
        delay(20); // Adjust delay for smoother/slower motion
    }
}

void right()
{
    int startPos22 = servo2.read(); // Get the current position of servo2
    for (int pos = startPos22; pos <= 180; pos += 1) { // Move from current position to 180 degrees
        servo2.write(pos);
        delay(20); // Adjust delay for smoother/slower motion
    }
}

void straight()
{
    int startPos1 = servo1.read(); // Get the current position of servo1
    for (int pos = startPos1; pos >= 0; pos -= 1) { // Move servo1 to 0 degrees
        servo1.write(pos);
        delay(20); // Adjust delay for smoother/slower motion
    }
    int startPos2 = servo2.read(); // Get the current position of servo2
    for (int pos = startPos2; pos >= 0; pos -= 1) { // Move servo2 to 0 degrees
        servo2.write(pos);
        delay(20); // Adjust delay for smoother/slower motion
    }
}






// // Servo Functions
// void left()
// {
//     servo1.write(180);
//     //servo2.write(0);
// }

// void right()
// {
//     //servo1.write(180);
//     servo2.write(180);
// }

// void straight(){
//     servo1.write(0);
//     servo2.write(0);
// }
// Reset Process Function
void resetProcess()
{
    timecounter = 0;
    lock = false;
    stateprocess = 4;
}
