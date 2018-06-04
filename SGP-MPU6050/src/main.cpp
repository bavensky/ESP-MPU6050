#include <Arduino.h>
#include <Wire.h>

const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gForceX, gForceY, gForceZ, rotX, rotY, rotZ;

#define A_R 16384.0 // 32768/2
#define G_R 131.0   // 32768/250

float Acc[2];
float Gy[3];
float Angle[3];
String value;
long time_prev;
float dt;

void dataReceiver();
void processData();
void debugFunction(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ);

void setup()
{
    Serial.begin(115200);
    Wire.begin(4, 5);
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); 
    Wire.write(0);    
    Wire.endTransmission(true);
}

void loop()
{
    dataReceiver();
    // debugFunction(AcX, AcY, AcZ, GyX, GyY, GyZ);
    // delay(200);
}

//###################################################################################
void dataReceiver()
{
    // Wire.beginTransmission(MPU);
    // Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU, 14, true);      // request a total of 14 registers
    // AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    // AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    // AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    // GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    // GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    // GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
    Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;

    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
    Gy[0] = GyX / G_R;
    Gy[1] = GyY / G_R;
    Gy[2] = GyZ / G_R;

    dt = (millis() - time_prev) / 1000.0;
    time_prev = millis();

    //  Complementary
    Angle[0] = 0.98 * (Angle[0] + Gy[0] * dt) + 0.02 * Acc[0];
    Angle[1] = 0.98 * (Angle[1] + Gy[1] * dt) + 0.02 * Acc[1];

    //  Integration
    Angle[2] = Angle[2] + Gy[2] * dt;

    value = "90, " + String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90";
    Serial.println(value);


    // processData();
}

void processData()
{
    Serial.print("AcX : ");
    Serial.print(AcX);
    Serial.print("\tAcY : ");
    Serial.print(AcY);
    Serial.print("\tAcZ : ");
    Serial.println(AcZ);

    //AFS_SEL : 0   ||  Full Scale Range : ±2g  ||  LSB Sensitivity :16384 LSB/g
    gForceX = AcX / 16384.0;
    gForceY = AcY / 16384.0;
    gForceZ = AcZ / 16384.0;

    // FS_SEL : 0   ||  Full Scale Range : ± 250 °/s    ||  LSB Sensitivity : 131 LSB/°/s
    rotX = GyX / 131.0;
    rotY = GyY / 131.0;
    rotZ = GyZ / 131.0;
}

void debugFunction(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ)
{
    // Print the MPU values to the serial monitor
    // Serial.print("Accelerometer: ");
    // Serial.print("X=");
    // Serial.print(gForceX);
    // Serial.print("|Y=");
    // Serial.print(gForceY);
    // Serial.print("|Z=");
    // Serial.println(gForceZ);
    Serial.print("Gyroscope:");
    Serial.print("X=");
    Serial.print(rotX);
    Serial.print("|Y=");
    Serial.print(rotY);
    Serial.print("|Z=");
    Serial.println(rotZ);
}

//###################################################################################