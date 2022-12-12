//libraries needed for calculations
#include <BasicLinearAlgebra.h>
#define _USE_MATH_DEFINES
using namespace BLA;
//libraries needed for sensor data collection
#include <ros.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <std_msgs/String.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
// ros::NodeHandle nh;
// std_msgs::String str_msg;
// ros::Publisher next_sawyer_loc("next_sawyer_loc", &str_msg);

const int T = 10;
double delta = 0.1; //this controls the amount of time between measurements (in seconds)
double current_time_step = 1;

//create matrices and vectors
double qW {1}, qX {0}, qY {0}, qZ {0}, qN {};
Matrix <3,T> imuAccelerations;
Matrix <3,T> spatialAccelerations;
Matrix <3,T> spatialVelocities;
Matrix <3,T> spatialPositions;

//test values
bool test = false;
// identity matrix -> quaternion (0,0,0,1) 
// and for example case (-0.5, 0.5, 0.5, 0.5)

// other test rotation matrix quaternion:
// double qX_test = -0.5; 
// double qY_test = 0.5;
// double qZ_test = 0.5;
// double qW_test = 0.5;

//identity matrix quaternion:
double qX_test = 0; 
double qY_test = 0;
double qZ_test = 0;
double qW_test = 1;

double linearaccel_X = 0.0;
double linearaccel_Y = 0.0; 
double linearaccel_Z = 1;

#pragma Matrix Functions
Matrix <3> get_matrix_column(Matrix<3,T> matrix, int col)
{
 Matrix<3> vector = {matrix(0,col), matrix(1,col), matrix(2,col)};
 return vector;
}
 
void set_matrix_column(String matrix_name, int col, double value1, double value2, double value3)
{
 if (matrix_name=="imuAccelerations"){
   imuAccelerations(0,col) = value1;
   imuAccelerations(1,col) = value2;
   imuAccelerations(2,col) = value3;
 } else if (matrix_name=="spatialAccelerations"){
   spatialAccelerations(0,col) = value1;
   spatialAccelerations(1,col) = value2;
   spatialAccelerations(2,col) = value3;
 } else if (matrix_name=="spatialVelocities"){
   spatialVelocities(0,col) = value1;
   spatialVelocities(1,col) = value2;
   spatialVelocities(2,col) = value3;
 } else if (matrix_name=="spatialPositions"){
   spatialPositions(0,col) = value1;
   spatialPositions(1,col) = value2;
   spatialPositions(2,col) = value3;
 }
}

Matrix<3> scalar_vector_mul(Matrix<3> vector, double num)
{
 Matrix<3> matrix;
 for (int i {}; i<3; i++)
 {
   matrix(i) = vector(i) * num;
 }
 return matrix;
}
 
void display_vector(Matrix<3> vector)
{
 for(int i{}; i<3; i++)
 {
   Serial.print(vector(i), 6);
   if (i < 2)
   {
    Serial.print(",");
   }
 }
 Serial.println(";");
}

void display_matrix(Matrix<3, 3> matrix) {
  for (int i {}; i < 3; i++)  {
    for (int j {}; j < 3; j++)  {
      Serial.print(matrix(i,j), 6);
      Serial.print("\t");
    }
    Serial.println();
  }
Serial.println("=================================================");
}

Matrix <3,3> quaternion_to_rotation_matrix(imu::Quaternion quat)
{
//rotation matrix calculations
double qX = qX_test;
double qY = qY_test;
double qZ = qZ_test;
double qW = qW_test;

if (!test){
  qX = quat.x();
  qY = quat.y();
  qZ = quat.z();
  qW = quat.w();

  if (qW == 0) {
    qW = 1;
  }
}
qN = sqrt((qW * qW) + (qX * qX) + (qY * qY)+ (qZ * qZ));
qW /= qN;
qX /= qN;
qY /= qN;
qZ /= qN;

Matrix <3,3> rotMatrix;
rotMatrix(0,0) = (1 - (2 * qY * qY) - (2 * qZ * qZ));
rotMatrix(0,1) = ((2 * qX * qY) - (2 * qZ * qW));
rotMatrix(0,2) = ((2* qX * qZ) + (2 * qY * qW));
rotMatrix(1,0) = ((2 * qX * qY) + (2 * qZ * qW));
rotMatrix(1,1) = (1 - (2 * qX * qX) - (2 * qZ *qZ));
rotMatrix(1,2) = ((2 * qY * qZ) - (2 * qX * qW));
rotMatrix(2,0) = ((2 * qX * qZ) - (2 * qY * qW));
rotMatrix(2,1) = ((2 * qY * qZ) + (2 * qX * qW));
rotMatrix(2,2) = (1 - (2 * qX * qX) - (2 * qY * qY));
return rotMatrix;
}
#pragma endregion
 
double norm(Matrix <3> x) {
  return sqrt(sq(x(0)) + sq(x(1)) + sq(x(2)));
}

Matrix<3> threshold(Matrix<3> x) {
  double thresh = 1.0;
  
  for (int i {}; i < 3; i++) {
    if (x(i) < thresh) {
      x(i) = 0;
    }
  }
  return x;
}

void setup(void)
{
  // nh.initNode();
  // nh.advertise(next_sawyer_loc);
  Serial.begin(9600);
  // Serial.println("Orientation Sensor Test"); Serial.println("");
  pinMode(LED_BUILTIN, 13);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  // initialize spatial values
  spatialVelocities(0, 0) = 0;
  spatialVelocities(1, 0) = 0;
  spatialVelocities(2, 0) = 0;
  spatialPositions(0, 0) = 0;
  spatialPositions(1, 0) = 0;
  spatialPositions(2, 0) = 0;
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop(void)
{
//  Serial.print("current time step: ");
//  Serial.println(current_time_step);
 
//  digitalWrite(LED_BUILTIN, HIGH);

  /* Get a new sensor event */
 sensors_event_t event;
 bno.getEvent(&event);

 imu::Quaternion quat = bno.getQuat();
 Matrix <3,3> rotMatrix = quaternion_to_rotation_matrix(quat);
//  display_matrix(rotMatrix);
 
 //get linear imu acceleration
 imu::Vector<3> temp = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

 Matrix <3> imu_linearaccel;
 imu_linearaccel(0) = temp[0];
 imu_linearaccel(1) = temp[1];
 imu_linearaccel(2) = temp[2];
 imu_linearaccel = threshold(imu_linearaccel);

 if(test){
  imu_linearaccel(0)= linearaccel_X;
  imu_linearaccel(1)= linearaccel_Y;
  imu_linearaccel(2)= linearaccel_Z;
 }
//  Serial.println("Current imu Acceleration");
//  display_vector(imu_linearaccel);
 
 Matrix <3> current_spatial_acceleration = rotMatrix * imu_linearaccel;
//  Serial.println("Current Spatial Acceleration:");
//  display_vector(current_spatial_acceleration);

 int idx = (current_time_step - 1) % 10;
 // Matrix <3> prev_imu_acceleration = get_matrix_column(imuAccelerations, idx);
 // Matrix <3> prev_spatial_acceleration = rotMatrix * prev_imu_acceleration;

 Matrix<3> prev_spatial_velocity = get_matrix_column(spatialVelocities, idx);
 // display_vector(prev_spatial_velocity);
//  display_vector(scalar_vector_mul(current_spatial_acceleration, delta));
 Matrix<3> current_spatial_velocity = prev_spatial_velocity + scalar_vector_mul(current_spatial_acceleration, delta);
//  Serial.println("Current spatial velocity:");
//  display_vector(current_spatial_velocity);

 Matrix<3> prev_spatial_pos = get_matrix_column(spatialPositions, idx);
 Matrix<3> current_spatial_pos = prev_spatial_pos + scalar_vector_mul(prev_spatial_velocity, delta); 
//  Serial.println("Current Spatial Position:");
//  display_vector(current_spatial_pos);

 // flush the velocity to 0
 if (current_time_step % 20 == 0) {
   current_spatial_velocity(0) = 0;
   current_spatial_velocity(1) = 0;
   current_spatial_velocity(2) = 0;
   digitalWrite(LED_BUILTIN, LOW);
  //  Serial.println("======PUBLISH======");
  //  Serial.println("Current Spatial Position:");
   Serial.print(current_time_step, ",");
   display_vector(current_spatial_pos);
   delay(1000);
  //  String test_str = String(current_spatial_pos(0),5)+' '+String(current_spatial_pos(1),5)+' '+String(current_spatial_pos(2),5);
   String test_str = "hello_world";
  //  Serial.print("String Length:");
  //  Serial.println(current_spatial_pos(0),6);
  //  char char_test[25] = {test_str.c_str()};
  //  str_msg.data = char_test; 
  //  next_sawyer_loc.publish(&str_msg);
 } else {
   digitalWrite(LED_BUILTIN, HIGH);
 }

 //store current imu accelerations to be used in next iteration calculations
 int next_idx = current_time_step % 10;
 set_matrix_column("imuAccelerations", next_idx, imu_linearaccel(0), imu_linearaccel(1), imu_linearaccel(2));
 set_matrix_column("spatialAccelerations", next_idx, current_spatial_acceleration(0), current_spatial_acceleration(1), current_spatial_acceleration(2));
 set_matrix_column("spatialVelocities", next_idx, current_spatial_velocity(0), current_spatial_velocity(1), current_spatial_velocity(2));
 set_matrix_column("spatialPositions", next_idx, current_spatial_pos(0), current_spatial_pos(1), current_spatial_pos(2));

 current_time_step += 1;

 delay(1000 * delta);
}
