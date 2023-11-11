#define ENCAtilt 2 // Yellow
#define ENCBtilt 3 // Green
#define ENCApan 18 // Yellow
#define ENCBpan 19 // Green
#define rev1 700 //number of ticks per one revolution


const int pinPwmTilt = 9;
const int pinPwmPan = 6;
// DIR pins for motor.
const int pinDirTilt = 8;
const int pinDirPan = 7;

int posTilt = 0;
int posPan = 0;
float angTilt = 0;
float angPan = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENCAtilt,INPUT);
  pinMode(ENCBtilt,INPUT);
  pinMode(ENCApan,INPUT);
  pinMode(ENCBpan,INPUT);
  pinMode(pinPwmTilt, OUTPUT);
  pinMode(pinDirTilt, OUTPUT);
  pinMode(pinPwmPan, OUTPUT);
  pinMode(pinDirPan, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCAtilt),readEncoderTilt,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCApan),readEncoderPan,RISING);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  readEncoderTilt();
  readEncoderPan();
  Serial.println(posTilt);
  Serial.println(posPan);
  Serial.println();
  //analogWrite(pinPwmTilt, 20);
  //digitalWrite(pinDirTilt, HIGH);
  //analogWrite(pinPwmPan, 0);
  delay(1000);
  //analogWrite(pinPwmTilt, 20);
  //digitalWrite(pinDirTilt, LOW);

  Serial.println(posTilt);
  Serial.println(posPan);
  Serial.println();

  delay(500);
  
}

void readEncoderTilt(){
  int b = digitalRead(ENCBtilt);
  if(b > 0){
    posTilt++;
  }
  else{
    posTilt--;
  }
  angTilt = (float)posTilt*360/700;
}

void readEncoderPan(){
  int b = digitalRead(ENCBpan);
  if(b > 0){
    posPan++;
  }
  else{
    posPan--;
  }
  angPan = (float)posPan*360/700;
}
