// importing ROS libraries
#include <ros.h>
#include <std_msgs/String.h>

// creating a ROS node object
ros::NodeHandle  nh;

// defining the datatype for rosserial communication
std_msgs::String str_data;
ros::Publisher ur_data_pub("ur_data", &str_data);

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

# define toggle_menu 12

// defining print and array length macro
#define sp(p) Serial.println(p)
#define arrLen(arr) sizeof(arr)/sizeof(arr[0])


// variables for potentiometer
int potPins[] = {j0_x, j0_y, p0, j1_x, j1_y, p1};
int potValues[6];

// variables for buttons
int buttonPins[] = {s0, s1, s2, s3};
int buttonValues[4];

// variables 
const int numReadings = 10;      // Number of readings to average
const int minAngle = -10;        // Minimum joystick angle in degrees
const int maxAngle = 10;         // Maximum joystick angle in degrees

// variables for filtering
int readings[numReadings][6];       // Array to store the readings
int currentIndex[6] = {0, 0, 0, 0, 0, 0};            // Current index in the array
float smoothedValue[6] = {0,0,0,0,0,0};         // Smoothed joystick value
float averageValue[6];

float transferValue[6];

int insig_fig = 100; //for removing unwanted percision
int input_offset= 0;
int input_max = 4000;
int output_range = 100;

// variables for zero set
int zeroCount=0;
int zeroCountMax=150;

float zeroPoints[6]={0,0,0,0,0,0};
float dataMin[6] = {4096,4096,4096,4096,4096,4096};
float dataMax[6] = {0,0,0,0,0,0};

// variables for smoothening
float alpha = 0.06;
float prevPotValues[6];
float alphaPotValues[6];



// Functions for General Tasks
void initializePins(int* pinDef){
  for(int i=0; i< sizeof(pinDef)/sizeof(pinDef[0]); i++){
    pinMode(pinDef[i], INPUT);
  }
}//okay1

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

void viewMatrix(){
  for( int i=0; i < 10; i++){ 
    for( int j=0; j< 6; j++){
      Serial.print(readings[i][j]); Serial.print(",");
    }
    Serial.println("");    
  }
}


// functions for input Pins
void addPotData(){
  for(int i=0; i<6; i++){
    potValues[i] = map(analogRead(potPins[i]),0,4096,0,100);
  }
}

void addButtonData(){
  for(int i=0; i<arrLen(buttonValues); i++){
    buttonValues[i] = (analogRead(buttonPins[i])>500);
  }
}//okay8

// functions for filtering
void initAlphaData(){
  for(int i=0; i<6;i++){
    prevPotValues[i]=transferValue[i];
    alphaPotValues[i]=transferValue[i];  
  }
}//okay4

void addNewTransferValues(){
  for(int i=0; i<6; i++){
    alphaPotValues[i] = (1-alpha)*prevPotValues[i] + alpha*transferValue[i];
    prevPotValues[i] = alphaPotValues[i] ;   
  }
}//okay10


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
}//okay4

void readSmoothData(){
  for (int i = 0; i < 6; i++) {
    // Read the analog value from the potentiometer
    //potValues[i] = map(int(analogRead(potPins[i])/insig_fig)*insig_fig+input_offset,0,input_max, 0,output_range);
    potValues[i] = analogRead(potPins[i]);
    //Serial.print(   ((analogRead(potPins[i])-1830) / 40.95 )+50);Serial.print(" ,");
    // Smooth the joystick value
    float smValue = smoothJoystickValue(potValues[i], i);

    // Print the angle and linear value
    transferValue[i] = smValue;
  }
}//okay3

void getMinMax(){
  for (int i = 0; i < 6; i++) {
    // Read the analog value from the potentiometer
    //potValues[i] = map(int(analogRead(potPins[i])/insig_fig)*insig_fig+input_offset,0,input_max, 0,output_range);
    potValues[i] = analogRead(potPins[i]);
    //Serial.print(   ((analogRead(potPins[i])-1830) / 40.95 )+50);Serial.print(" ,");
    // Smooth the joystick value
    if(potValues[i] <= dataMin[i] ){
        dataMin[i] = potValues[i];     
      }else if(potValues[i] > dataMax[i] ){
        dataMax[i] = potValues[i];
      }
  }
}//okay3

void initializeSmoothData(){
  zeroCount = 0;
  Serial.println(" MinMax Set");
  while(zeroCount < zeroCountMax/2){
    getMinMax();
    zeroCount++;
    Serial.print(zeroCount);Serial.print("...");
    delay(100);
  }
  delay(3000);
  Serial.println( "Mean Set");
  while(zeroCount < zeroCountMax){
    readSmoothData();
    zeroCount++;
    Serial.print(zeroCount);Serial.print("..");
    delay(100);
  }
  Serial.println("Starting values Set");
  while(zeroCount < zeroCountMax+30){
    readSmoothData();
    zeroCount++;
    Serial.print(zeroCount);Serial.print("..");
    delay(100);
  }
}//okay2

void setZero(){
  for( int i=0; i < 6; i++){ 
    for( int j=0; j< 10; j++){
      zeroPoints[i] += readings[j][i];
    }
    zeroPoints[i] = zeroPoints[i] / 10;    
  }
}//okay3

void setTransferData(){
  for(int i=0; i<6; i++){
    transferValue[i] = ((transferValue[i] - zeroPoints[i])*100/(dataMax[i]-dataMin[i]) + 50);
//    if(transferValue[i] > dataMin[i] && transferValue[i] < dataMax[i]){
//      transferValue[i] = zeroPoints[i]; 
//    }
  }
}//okay9


//rossrial parts
String floatToString( float value){
  return String(int(value));
}

// Function to convert int value to string
String intToString(int value){
  return String(value);
}

// Function to send data to rosserial
void sendSensorData(float* floats, int* ints){
  // Clear the message data
  String msg = "";

  // Convert float values to string and append to the message
  for (int i = 0; i < 6; i++)
  {
    String floatValue = floatToString(floats[i]);
    msg += floatValue;
    msg += " ";
  }

  // Convert int values to string and append to the message
  for (int i = 0; i < 4; i++)
  {
    String intValue = intToString(ints[i]);
    msg += intValue;
    msg += " ";
  }//okay11
  
  // Length (with one extra character for the null terminator)
  int str_len = msg.length() + 1; 
  
  // Prepare the character array (the buffer) 
  char char_array[str_len];
  
  // Copy it over 
  msg.toCharArray(char_array, str_len);
  
  //Serial.println(char_array);

  send_ros(char_array);
}

void init_ros(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(ur_data_pub);
}//okay5

void send_ros(char* hello){
  str_data.data = hello;
  ur_data_pub.publish( &str_data );
  nh.spinOnce();
}



void setup(){

  Serial.begin(115200);

  initializePins(potPins); 

  initializePins(buttonPins);
  
  analogReadResolution(12);

  Serial.println(" Starting Calibration");
  delay(3000);
  
  initializeSmoothData();
  
  viewData("TransferValue - ", transferValue);Serial.println();delay(10000);  

  setZero();
  
  viewMatrix();delay(10000);

viewData("ZeroPoints - ", zeroPoints);Serial.println();delay(1000);  
viewData("min- ", dataMin);Serial.println();delay(1000);  
viewData("max- ", dataMax);Serial.println();delay(10000);  

  
  initAlphaData();
  init_ros();

}

void loop(){

  readSmoothData();

  addButtonData();

  setTransferData();

  addNewTransferValues();
  
//  viewData("", transferValue);
//  viewData("", dataMin);
//  viewData("", dataMax);
//  viewData("", zeroPoints);
  viewData("", alphaPotValues);
  viewData("", buttonValues);
  Serial.println();

//  Serial.print(analogRead(12));Serial.print("  ");Serial.println(analogRead(26));

  sendSensorData(alphaPotValues, buttonValues);

  delay(10);

}
