#include <ros.h>
// #include <std_msgs/Int32.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/MagneticField.h>
#include <Arduino_LSM9DS1.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include "MahonyAHRS.h"

Mahony filter;

// ROS node handle
ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
geometry_msgs::Vector3 orient;
tf::TransformBroadcaster broadcaster;

ros::Publisher imu_pub("imu_data", &orient);

char frameid[] = "/base_link";
char child[] = "/imu_frame";


unsigned long microsPerReading, microsPrevious;
float accl_scale, gyro_scale;


void setup()
{
  // Initialize ROS communication
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(imu_pub);

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
/*****************  For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.   ****************
*****************       Copy/Replace the lines below by the code output of the program            ****************/
   IMU.setAccelFS(3);           
   IMU.setAccelODR(5);            
   IMU.setAccelOffset(-0.012852, 0.004973, -0.025369);  // 0,0,0 = uncalibrated
   IMU.setAccelSlope (0.993130, 1.004145, 0.999773);  // 1,1,1 = uncalibrated

   // Accelerometer code
   IMU.setAccelFS(3);
   IMU.setAccelODR(5);
   IMU.setAccelOffset(-0.013971, 0.003563, -0.022866);
   IMU.setAccelSlope (0.996939, 1.003876, 0.998388);

/*******   The gyroscope needs to be calibrated. Offset controls drift and Slope scales the measured rotation angle  *********
*****************   Copy/Replace the lines below by the output of the DIY_Calibration_Gyroscope sketch   ********************/
   IMU.setGyroFS(3);    
   IMU.setGyroODR(5);
   IMU.setGyroOffset (0.985596, 0.954041, -1.142609);  // = uncalibrated
   IMU.setGyroSlope  (1, 1, 1);  // = uncalibrated
  
   // Gyroscope code
   IMU.setGyroFS(2);
   IMU.setGyroODR(5);
   IMU.setGyroOffset (0.927856, 0.868988, -0.973053);
   IMU.setGyroSlope (1.143575, 2.141141, 1.124757);

/*****************   For a proper functioning of the compass the magnetometer needs to be calibrated    ********************
*****************   Replace the lines below by the output of the DIY_Calibration_Magnetometer sketch   ********************/
   IMU.setMagnetFS(0);  
   IMU.setMagnetODR(8); 
   IMU.setMagnetOffset(32.225952, 2.850952, 11.129150);  //  uncalibrated
   IMU.setMagnetSlope (1.449257, 1.304183, 1.386317);  //  uncalibrated

   // Magnetometer code
   IMU.setMagnetFS(0);
   IMU.setMagnetODR(8);
   IMU.setMagnetOffset(28.909302, 3.363037, 11.284790);
   IMU.setMagnetSlope (1.209661, 1.217642, 1.260209);

  // The slowest ODR determines the sensor rate, Accel and Gyro share their ODR
  float sensorRate = min(IMU.getGyroODR(),IMU.getMagnetODR());

  filter.begin(sensorRate);
  
  Serial.print ("Accelerometer sample rate = ");
  Serial.print (IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tz");
  Serial.print ("Gyroscope sample rate = ");
  Serial.print (IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tz");
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
  delay(5000);
  
  microsPerReading = 1000000 / sensorRate;
  microsPrevious = micros();
}

void loop()
{
  float ax, ay, az; //acclerometer reading is already in G / sec
  float gx, gy, gz; //Gyroscope reading is already in degrees / sec
  float mx, my, mz;

   // values for acceleration & rotation:
   float xAcc, yAcc, zAcc;
   float xGyro, yGyro, zGyro;
   float xMag, yMag, zMag;

  float acceleration[3], dps[3], magneticField[3];
  
  float roll, pitch, heading;
  
  unsigned long microsNow;
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    //if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
/*
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);
*/
   if (IMU.accelerationAvailable() &&
       IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
     // read accelerometer & gyrometer:
      IMU.readAcceleration(xAcc, yAcc, zAcc);
      IMU.readGyroscope(xGyro, yGyro, zGyro);
      IMU.readMagneticField(xMag, yMag, zMag);
/*  
      IMU.readAcceleration(xAcc, yAcc, zAcc);
      acceleration[0] = -yAcc;
      acceleration[1] = -xAcc;
      acceleration[2] = zAcc;
      IMU.readGyroscope(xGyro, yGyro, zGyro);
      dps[0] = -yGyro; 
      dps[1] = -xGyro; 
      dps[2] = zGyro;
      IMU.readMagneticField(xMag, yMag, zMag);
      magneticField[0] = xMag;  //magnetic North
      magneticField[1] = -yMag;  //West
      magneticField[2] = -zMag;  //Up
  
     xAcc = acceleration[0];
     yAcc = acceleration[1];
     zAcc = acceleration[2];
  
     xGyro = dps[0];
     yGyro = dps[1];
     zGyro = dps[2];
  
     xMag = magneticField[0];
     yMag = magneticField[1];
     zMag = magneticField[2];
*/  
    filter.update(yGyro, xGyro , zGyro, yAcc, xAcc, zAcc, yMag, -xMag, zMag); //for all 3
    //filter.update(xGyro, yGyro , zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag); //for all 3
    //filter.update(gx, gy , gz, ax, ay, az, mx, my, mz); //for all 3
    //filter.update(gx * DEG_TO_RAD, -gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, -ay, az, -mx, my, -mz); //for all 3
    //filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); //for all 3
    //filter.updateIMU(gx, gy, gz, ax, ay, az);//only for accl, gyro
  
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
    Serial.print("Orientation: ");
    Serial.print (heading);
    Serial.print(" ");
    Serial.print (pitch);
    Serial.print(" ");
    Serial.println(roll);
    
    orient.x = roll * 180 / M_PI;
    orient.y = pitch * 180 / M_PI;
    orient.z = heading * 180 / M_PI;
    
    imu_pub.publish(&orient);
    
    float qx, qy, qz, qw;
    filter.getQuaternion(&qx, &qy, &qz, &qw);
    
    t.header.frame_id = frameid;
    t.child_frame_id = child;
    t.transform.translation.x = 1.0;
    t.transform.rotation.x = qx;
    t.transform.rotation.y = qy;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    
    microsPrevious = microsPrevious + microsPerReading;
    nh.spinOnce();
    }
  }
}
