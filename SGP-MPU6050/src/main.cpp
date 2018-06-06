#include <Arduino.h>
#include <Wire.h>

const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

#define A_R 16384.0 // 32768/2
#define G_R 131.0   // 32768/250

float Acc[2];
float Gy[3];
float Angle[3];

long time_prev;
float dt;

String value;

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

    //  Debug data
    Serial.printf("Gy : %2f     Gx : %2f     Gz : %2f \r\n", Angle[0],  Angle[1],  Angle[2]);
}
