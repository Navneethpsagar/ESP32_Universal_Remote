//----------Import Libraries----------//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#include <ros.h>
#include <std_msgs/Int32.h>

//----------Pin Definitions----------//
#define j0_x 36  // Assuming the analog pin is A0

//----------Variable Definitions----------//
#define lcd_ch 20
#define lcd_ln 4

#define tx_ce 4
#define tx_csn 5

//----------Object Definitions----------//
LiquidCrystal_I2C lcd(0x27, lcd_ch, lcd_ln);

RF24 radio(tx_ce, tx_csn);

//----------ros node----------//
ros::NodeHandle nh;
std_msgs::Int32 potentiometer_msg;
ros::Publisher potentiometer_pub("potentiometer", &potentiometer_msg);

//----------Setup----------//
void setup() {
  //----------Serial----------//
  Serial.begin(115200);

  //----------LCD----------//
  lcd.init();
  lcd.backlight();

  //----------Radio----------//
  if (!radio.begin()) {
    lcd.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  lcd.println("Radio is on");
  delay(10000);

  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(float));

  nh.initNode();
  nh.advertise(potentiometer_pub);
}

void loop() {
  int analogValue = analogRead(j0_x);

  //----------SerialPrintValues----------//
  Serial.print("j0_x:");
  Serial.print(analogValue);

  //----------LCDPrintValues----------//
  static int previousValue = -1;
  if (analogValue != previousValue) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("j0_x:");
    lcd.print(analogValue);
    previousValue = analogValue;
  }

  //----------ros publish----------//
  potentiometer_msg.data = analogValue;
  potentiometer_pub.publish(&potentiometer_msg);
  nh.spinOnce();

  //----------Delay----------//
  delay(10);
}
