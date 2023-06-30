/*
 * Displays text sent over the serial port (e.g. from the Serial Monitor) on
 * an attached LCD.
 * YWROBOT
 *Compatible with the Arduino IDE 1.0
 *Library version:1.1
 */
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

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

#define lcd_heading(heading) lcd.setCursor(3,0);lcd.print(heading)
#define lcd_line1(line1) lcd.setCursor(0,1);lcd.print(line1)
#define lcd_line2(line2) lcd.setCursor(0,2);lcd.print(line2)
#define lcd_line3(line3) lcd.setCursor(0,3);lcd.print(line3)

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int col = 20;
int row = 4;

void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Serial.begin(115200);

  pinMode(s0,INPUT);
  pinMode(s1,INPUT);

  pinMode(s2,INPUT);
  pinMode(s3,INPUT);

  lcd.clear();
  mainMenu();
}

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

int secButtonPushCounter = 0;   // counter for the number of button presses
int secButtonState = 0;         // current state of the button
int secLastButtonState = 0;     // previous state of the button

int menuOption=0;

void mainMenu(){
  lcd.clear();
  
  lcd_heading("WELCOME!");

  lcd_line1("1. ROS Mode!");

  lcd_line2("2. NRF24 Mode!");

  lcd_line3("3. Rpi Display!");

  lcd.setCursor(18, buttonPushCounter+1);
  lcd.print("<=");
  
}

void rosMenu(){
  lcd.clear();
  
  lcd_heading("ROS Mode!");

  lcd_line1("1. Rosserial?");

  lcd_line2("2. RosWifi");

  lcd_line3("3. Back");

  lcd.setCursor(18, buttonPushCounter+1);
  lcd.print("<=");
  
}

void nrfMenu(){
  lcd.clear();
  
  lcd_heading("NRF24 Mode!");

  lcd_line1("1. Transmit");

  lcd_line2("2. Receive");

  lcd_line3("3. Back");

  lcd.setCursor(18, buttonPushCounter+1);
  lcd.print("<=");
  
}

void rpiMenu(){
  lcd.clear();
  
  lcd.setCursor(3,0);
  lcd.print("RPI Display Mode!");

  lcd.setCursor(0,1);
  lcd.print("1. Visualize");

  lcd.setCursor(0,2);
  lcd.print("2. Troubleshoot");

  lcd.setCursor(0,3);
  lcd.print("3. Back");

  lcd.setCursor(18, buttonPushCounter+1);
  lcd.print("<=");
  
}

void changeSelection(){
  buttonState = digitalRead(s0);
  secButtonState = digitalRead(s3);
  

  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter = (buttonPushCounter + 1) % 3;
      lcdMenu();
    }// Delay a little bit to avoid bouncing
    delay(50);
  }
  lastButtonState = buttonState;

  if (secButtonState != secLastButtonState) {
    if (secButtonState == HIGH) {
      if(buttonPushCounter == 2 && menuOption!=0){
        menuOption = 0;
      }else{
        menuOption=buttonPushCounter+1;
      }
      buttonPushCounter = 0;
      lcdMenu();
    }// Delay a little bit to avoid bouncing
    delay(50);
  }
  secLastButtonState = secButtonState;
}

void lcdMenu(){
  switch(menuOption){
    case 1: rosMenu();break;
    case 2: nrfMenu();break;
    case 3: rpiMenu();break;

    default: mainMenu();break;
    
  }
}
void loop()
{
  changeSelection();

  Serial.println(buttonPushCounter);
  
  delay(100);
}
