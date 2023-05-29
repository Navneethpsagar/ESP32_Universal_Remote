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




void setup() {
  Serial.begin(115200);
  
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
  
}
 
void loop() {
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
}
