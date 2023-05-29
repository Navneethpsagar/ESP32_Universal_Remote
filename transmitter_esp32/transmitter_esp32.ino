#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(5,4); // CE, CSN

const byte address[6] = "00001";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  float j0_x = 0;
  float j0_y = 125;
};

float j_prev=0;
float thru_put = 0.75;

Data_Package data; // Create a variable with the above structure

void setup() {
  radio.begin();
  Serial.begin(115200);

  pinMode(39,INPUT);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  // Send the whole data from the structure to the receiver
  float j_curr = analogRead(39);

  if(abs(j_curr-j_prev)< 8)
    j_curr = j_prev;
  Serial.print("a:");
  Serial.print(j_curr);
  Serial.print(",b:");
  Serial.print(data.j0_y);

  
  data.j0_x = j_prev*(1-thru_put) + j_curr*thru_put;
  j_prev = j_curr;

  Serial.print(",c:");
  Serial.println(data.j0_x);
  
  radio.write(&data, sizeof(Data_Package));
  delay(50);
  
}
