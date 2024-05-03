#include <Arduino.h>
#include <Wire.h>
#include <i2c_scanner.h>
#include <MPU9255.h>

#define g 9.81
#define magnetometer_cal 0.06 // magnetometer calibration

// Next Steps: 
// Optimize: Configure scaling and sensitivity to our Rover, and filter out spikes from readings

// MPU9255 I2C Address: 0x68
MPU9255 mpu;

// Process raw acceleration data
// Input = raw reading from the sensor, sensor_scale = selected sensor scale
// Output = acceleration in m/s^2
double process_acceleration(int input, scales sensor_scale)
{
  /*
  To get acceleration in 'g', each reading has to be divided by:
  -> 16384 for +- 2g scale (default scale)
  -> 8192 for +- 4g scale
  -> 4096 for +- 8g scale
  -> 2048 for +- 16g scale
  */
  double output = 1;

  if (sensor_scale == scale_2g)
  {
    output = input / 16384;
  }

  if (sensor_scale == scale_4g)
  {
    output = input / 8192;
  }

  if (sensor_scale == scale_8g)
  {
    output = input / 4096;
  }

  if (sensor_scale == scale_16g)
  {
    output = input / 2048;
  }

  output = output * g;

  return output;
}

// Process raw gyroscope data
// Input = raw reading from sensor, sensor_scale = selected sensor scale
// Output: angular velocity in degrees per second
double process_angular_velocity(int16_t input, scales sensor_scale) {
  /*
  To get angular velocity in dps (degrees per second), each reading has to be divided by: 
  -> 131 for +- 250 dps scale (default value)
  -> 65.5 for +- 500 dps scale
  -> 32.8 for +- 1000 dps scale
  -> 16.4 for +- 2000 dps scale
  */

 if (sensor_scale == scale_250dps) {
  return input/131;
 }

 if (sensor_scale == scale_500dps) {
  return input/65.5;
 }

 if (sensor_scale == scale_1000dps) {
  return input/32.8;
 }

 if (sensor_scale == scale_2000dps) {
  return input/16.4;
 }

 return 0;
}

// Process raw magnetometer data
// Input = raw reading from the sensor, sensitivity
// Output = magnetic flux density in uT (micro Teslas)
double process_magnetic_flux(int16_t input, double sensitivity) {
  /*
  To get magnetic flux density in uT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stormed in ROM), then multiplied by some number (calibration)
  and then divided by 0.6.
  (Faced North each axis should output around 31 uT without any metal / walls around)
  Note: This magnetometer has really low initial calibration tolerance: +- 500 LSB 
  Scale of the magnetometer is fixed -> +- 4800 uT
  */

  return (input*magnetometer_cal*sensitivity)/0.6;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22); // sda = GPIO_21 / scl = GPIO_22

  if (mpu.init())
  {
    Serial.println("Initialization failed");
  }
  else
  {
    Serial.println("Initialization successful");
  }
}

void loop()
{
  // Scanner();
  mpu.read_acc();  // Get data from the accelerometer
  mpu.read_gyro(); // Get data from the gyroscope
  mpu.read_mag();  // Get data from the magnetometer

  // Process and print accelerometer data
  Serial.print("AX: ");
  Serial.println(process_acceleration(mpu.ax, scale_2g));
  Serial.print("AY: ");
  Serial.println(process_acceleration(mpu.ay, scale_2g));
  Serial.print("AZ: ");
  Serial.println(process_acceleration(mpu.az, scale_2g));

  // Process and print gyroscope data
  Serial.print("GX: ");
  Serial.println(process_angular_velocity(mpu.gx, scale_250dps));
  Serial.print("GY: ");
  Serial.println(process_angular_velocity(mpu.gy, scale_250dps));
  Serial.print("GZ: ");
  Serial.println(process_angular_velocity(mpu.gz, scale_250dps));

  // Process and print magnetometer data
  Serial.print("MX: ");
  Serial.println(process_magnetic_flux(mpu.mx, mpu.mx_sensitivity));
  Serial.print("MY: ");
  Serial.println(process_magnetic_flux(mpu.my, mpu.my_sensitivity));
  Serial.print("MZ: ");
  Serial.println(process_magnetic_flux(mpu.mz, mpu.mz_sensitivity));

  delay(1000);
}
