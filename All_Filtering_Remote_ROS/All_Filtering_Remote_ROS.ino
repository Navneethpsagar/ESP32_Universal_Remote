#include "ros.h"
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle  nh;

std_msgs::Int32 data_msg;
sensor_msgs::Joy ur_data_msg;

ros::Publisher chatter2("ur_data2", &data_msg);
ros::Publisher joy_pub("esp_ur", &ur_data_msg);


//----------Pin Definitions----------//
#define j0_x 36
#define j0_y 39
#define p0 34

#define j1_x 35
#define j1_y 32
#define p1 33

#define s0 25
#define s1 26
#define s2 27
#define s3 14

#define sp(p) Serial.println(p)
#define arrLen(arr) sizeof(arr)/sizeof(arr[0])

// variables for potentiometer
int potPins[] = {j0_x, j0_y, p0, j1_x, j1_y, p1};
int potValues[6];

const int numReadings = 10;      // Number of readings to average
const int minAngle = -10;        // Minimum joystick angle in degrees
const int maxAngle = 10;         // Maximum joystick angle in degrees

int readings[numReadings][6];       // Array to store the readings
int currentIndex[6] = {0, 0, 0, 0, 0, 0};            // Current index in the array
float smoothedValue[6] = {0,0,0,0,0,0};         // Smoothed joystick value
float averageValue[6];

float transferValue[6];

int zeroCount=0;
int zeroCountMax=20;
float zeroPoints[6]={0,0,0,0,0,0};
float dataMin[6] = {1023,1023,1023,1023,1023,1023};
float dataMax[6] = {0,0,0,0,0,0};

float alpha = 0.06;
float prevPotValues[6];
float alphaPotValues[6];

// variables for buttons
int buttonPins[] = {s0, s1, s2, s3};
int buttonValues[4];


// Functions for General Tasks
void initializePins(int* pinDef){
  for(int i=0; i< sizeof(pinDef)/sizeof(pinDef[0]); i++){
    pinMode(pinDef[i], INPUT);
  }
}

void addPotData(){
  for(int i=0; i<6; i++){
    potValues[i] = analogRead(potPins[i]);
  }
}

void viewData(char* dataName, float* dataPoints){
  Serial.print(dataName);
  for(int i=0; i< 6; i++){
    Serial.print(dataPoints[i]);
    Serial.print(", ");
  }
}

void viewData(char* dataName, int* dataPoints){
  Serial.print(dataName);
  for(int i=0; i< 4; i++){
    Serial.print(dataPoints[i]);
    Serial.print(", ");
  }
}

void viewButtonData(char* dataName, int* dataPoints){
  Serial.print(dataName);
  for(int i=0; i< arrLen(dataPoints); i++){
    Serial.print(dataPoints[i]);
    Serial.print(", ");
  }
}



void initAlphaData(){
  for(int i=0; i<6;i++){
    prevPotValues[i]=transferValue[i];
    alphaPotValues[i]=transferValue[i];  
  }
}

void addNewTransferValues(){
  for(int i=0; i<6; i++){
    alphaPotValues[i] = (1-alpha)*prevPotValues[i] + alpha*transferValue[i];
    prevPotValues[i] = alphaPotValues[i] ;   
  }
}

// filtering

// Function to smooth the joystick readings
float smoothJoystickValue(int rawValue, int index) {
  // Subtract the oldest reading from the total
  //viewData("! - ", smoothedValue);Serial.println();
  smoothedValue[index] -= readings[currentIndex[index]][index];

  // Add the new reading to the total
  readings[currentIndex[index]][index] = rawValue;
  smoothedValue[index] += readings[currentIndex[index]][index];

  // Move to the next position in the array
  currentIndex[index] = (currentIndex[index] + 1) % numReadings;

  // Calculate the average value
  averageValue[index] = smoothedValue[index] / numReadings;
  //viewData("@ - ", averageValue);Serial.println();

  return averageValue[index];
}


void readSmoothData(){
  for (int i = 0; i < 6; i++) {
    // Read the analog value from the potentiometer
    potValues[i] = analogRead(potPins[i]);

    // Smooth the joystick value
    float smValue = smoothJoystickValue(potValues[i], i);

    // Print the angle and linear value
    transferValue[i] = smValue;
  }
}

void initializeSmoothData(){
  zeroCount = 0;
  while(zeroCount < zeroCountMax*2){
    readSmoothData();
    zeroCount++;
    Serial.print(zeroCount);Serial.print("->");
  }
  Serial.println();
}

void setZeroMinMax(){
  for( int i=0; i < 6; i++){ 
    for( int j=0; j< 10; j++){
      if(readings[j][i] <= dataMin[i] ){
        dataMin[i] = readings[j][i];     
      }else if(readings[j][i] > dataMax[i] ){
        dataMax[i] = readings[j][i];
      }
      zeroPoints[i] += readings[j][i];
    }
    zeroPoints[i] = zeroPoints[i] / 10;    
  }
}

void viewMatrix(){
  for( int i=0; i < 10; i++){ 
    for( int j=0; j< 6; j++){
      Serial.print(readings[i][j]); Serial.print(",");
    }
    Serial.println("");    
  }
}

void setTransferData(){
  for(int i=0; i<6; i++){
    if(transferValue[i] > dataMin[i] && transferValue[i] < dataMax[i]){
      transferValue[i] = zeroPoints[i]; 
    }
  }
}


// functions for button Pins
void addButtonData(){
  for(int i=0; i<arrLen(buttonValues); i++){
    buttonValues[i] = digitalRead(buttonPins[i]);
  }
}

void initRosNode(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(chatter2);  
  nh.advertise(joy_pub);  
  
}
void setup(){

  Serial.begin(9600);

  initializePins(potPins); 

  initializePins(buttonPins);
  
  analogReadResolution(10);

  initializeSmoothData();//viewData("TransferValue - ", transferValue);Serial.println();delay(10000);  

  setZeroMinMax();//viewMatrix();delay(10000);

  initAlphaData();

  initRosNode(); 

}
int i=0;
int j=5;
void loop(){

  readSmoothData();

  addButtonData();

  delay(10);

  setTransferData();

  addNewTransferValues();
  
//  viewData("", transferValue);
//  viewData("", dataMin);
//  viewData("", dataMax);
//  viewData("", zeroPoints);
  viewData("", alphaPotValues);
  viewData("", buttonValues);
  Serial.println();


  data_msg.data = 25*i*j;
  i = j+5;
  j=i-4;
//
//  // Convert the analog values to Joy message format (-1.0 to 1.0)
//  float x = map(x_axis, 0, 4095, -1, 1) * 0.5;  // Scale to -1.0 to 1.0
//  float y = map(y_axis, 0, 4095, -1, 1) * 0.5;
  ur_data_msg.buttons[0] = 1;
  ur_data_msg.buttons[1] = 0;

  ur_data_msg.axes[0] = -1;
  ur_data_msg.axes[1] = 0.5;

  
  joy_pub.publish( &ur_data_msg);
  nh.spinOnce();
}
