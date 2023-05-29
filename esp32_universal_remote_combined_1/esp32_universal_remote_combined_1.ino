
//----------Import Libraries----------//
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include <SPI.h>
#include "printf.h"
#include "RF24.h"




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

//----------Variable Definitions----------//
#define lcd_ch 20
#define lcd_ln 4

#define tx_ce 4
#define tx_csn 5

//----------Object Definitions----------//
// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
// set the LCD address to 0x27 for a lcd_ch chars and lcd_ln line display
LiquidCrystal_I2C lcd(0x27, lcd_ch, lcd_ln);  


// instantiate an object for the nRF24L01 transceiver
// using pin 7 for the CE pin, and pin 8 for the CSN pin
RF24 radio(tx_ce, tx_csn);  

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;


//----------Functions Definitions----------//




//----------Setup----------//
void setup() {

  //----------Serial----------//
  Serial.begin(115200);

  //----------LCD----------//
  // initialize the lcd 
  lcd.init(); 
  // turn on lcd backlight
  lcd.backlight();

  //----------pinModes----------//
  pinMode(j0_x,INPUT);
  pinMode(j0_y,INPUT);

  pinMode(j1_x,INPUT);
  pinMode(j1_y,INPUT);

  pinMode(p0,INPUT);
  pinMode(p1,INPUT);

  pinMode(s0,INPUT);
  pinMode(s1,INPUT);

  pinMode(s2,INPUT);
  pinMode(s3,INPUT);

  //----------Radio----------//
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    lcd.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  lcd.println("Radio is on");delay(10000);

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }


  
  
  
}
 
void loop() {


  //----------SerialPrintValues----------//
  Serial.print("j0_x:");Serial.print(analogRead(j0_x));
  Serial.print(",j0_y:");Serial.print(analogRead(j0_y));
  Serial.print(",j1_x:");Serial.print(analogRead(j1_x));
  Serial.print(",j1_y:");Serial.print(analogRead(j1_y)); 
  Serial.print(",p0:");Serial.print(analogRead(p0));
  Serial.print(",p1:");Serial.print(analogRead(p1)); 
  Serial.print(",s0:");Serial.print(analogRead(s0));
  Serial.print(",s1:");Serial.print(analogRead(s1)); 
  Serial.print(",s2:");Serial.print(analogRead(s2));
  Serial.print(",s3:");Serial.print(analogRead(s3)); 
  Serial.println();
  delay(0); 

  //----------LCDPrintValues
  // clear the screen
  lcd.clear();
  // set the lcd cursor
  lcd.setCursor(0,0);

  // print message
  lcd.print("j0_x:");
  lcd.print(analogRead(j0_x));
  lcd.print(",j0_y:");
  lcd.print(analogRead(j0_y));

  //----------delay----------//
  delay(10);
  
}
