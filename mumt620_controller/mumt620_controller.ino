#define XPIN A0
#define YPIN A1
#define R1 A2
#define R2 A3
#define R3 A4
#define R4 A5

//0~1023 to 0~250 (/4.1)

double xr, yr, br, r1, r2, r3, r4;
uint8_t radi = 0;
uint8_t thet = 0;
double pi = 3.1415926535897932;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  //recommended 115200
  pinMode(XPIN, INPUT);
  pinMode(YPIN, INPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
  pinMode(R3, INPUT);
  pinMode(R4, INPUT);
  pinMode(RED, OUTPUT);
  pinMode(GRE, OUTPUT);
  pinMode(BLU, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  xr = analogRead(XPIN)-544;
  yr = analogRead(YPIN)-537;
  r1 = analogRead(R1)/4.0;
  r2 = analogRead(R2)/4.0;
  r3 = analogRead(R3)/4.0;
  r4 = analogRead(R4)/4.0;
  car2polnorm(xr, yr, &radi, &thet);
  fsrthresh(&r1, &r2, &r3, &r4);
  communicate_char(radi, thet, r1, r2, r3, r4);
  //communicate_char(1, 2, 3, 4, 5, 6);
  //communicate_int(radi, thet, r1, r2, r3, r4);
  //communicate_raw(xr, yr, r1, r2, r3, r4);
  
}

void car2polnorm(double in1, double in2, uint8_t *radius, uint8_t *theta){
  //radius: 0~250 (/250*2pi)
  //theta: 0~250 (/250*2pi)

  //radius with hard limiter
  double r = sqrt( pow(in1,2) + pow(in2,2) )/2.92;
  if (r < 0) r = 0;
  if (r > 250) r = 250;
  *radius = uint8_t(r);

  //theta
  double a;
  if (in1==0 && in2==0){
    a = 0;
  }else{
    a = atan(in2/in1)/2/pi*250;
  }
  *theta = uint8_t(a);
}

void fsrthresh(double *in1, double *in2, double *in3, double *in4){
  double out1 = *in1;
  double out2 = *in2;
  double out3 = *in3;
  double out4 = *in4;
  if( abs(out1) > 250.0 ) out1 = out1 / abs(out1);
  if( abs(out1) < 12.5 ) out1 = 0;
  if( abs(out2) > 250.0 ) out2 = out2 / abs(out2);
  if( abs(out2) < 12.5 ) out2 = 0;
  if( abs(out3) > 250.0 ) out3 = out3 / abs(out3);
  if( abs(out3) < 12.5 ) out3 = 0;
  if( abs(out4) > 250.0 ) out4 = out4 / abs(out4);
  if( abs(out4) < 12.5 ) out4 = 0;
  *in1 = out1;
  *in2 = out2;
  *in3 = out3;
  *in4 = out4;
}

void communicate_char(uint8_t in1, uint8_t in2, double in3, double in4, double in5, double in6){
  uint8_t out1, out2, out3, out4, out5, out6;
  out3 = uint8_t(in3);
  out4 = uint8_t(in4);
  out5 = uint8_t(in5);
  out6 = uint8_t(in6);
  //min max 0~250
  Serial.print(char(255));  //reserved for order message
  Serial.print(char(in1));
  Serial.print(char(in2));
  Serial.print(char(out3));
  Serial.print(char(out4));
  Serial.print(char(out5));
  Serial.print(char(out6));
}

//debug only vvvvv
void communicate_int(uint8_t in1, uint8_t in2, double in3, double in4, double in5, double in6){
  Serial.print(int(255));
  Serial.print(" ");
  Serial.print(in1);
  Serial.print(" ");
  Serial.print(in2);
  Serial.print(" ");
  Serial.print(uint8_t(in3));
  Serial.print(" ");
  Serial.print(uint8_t(in4));
  Serial.print(" ");
  Serial.print(uint8_t(in5));
  Serial.print(" ");
  Serial.print(uint8_t(in6));
  Serial.println(" ");
}

void communicate_raw(double in1, double in2, double in3, double in4, double in5, double in6){
  Serial.print((int)in1);
  Serial.print(" ");
  Serial.print((int)in2);
  Serial.print(" ");
  Serial.print((int)analogRead(R1));
  Serial.print(" ");
  Serial.print((int)analogRead(R2));
  Serial.print(" ");
  Serial.print((int)analogRead(R3));
  Serial.print(" ");
  Serial.print((int)analogRead(R4));
  Serial.println(" ");
}
