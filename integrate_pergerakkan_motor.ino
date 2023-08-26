
int cw1 = 25;
int ccw1 = 23;
int cw2 = 27;
int ccw2 = 29;
int cw3 = 31;
int ccw3 = 33;
int cw4 = 37;
int ccw4 = 35;

int speed1 = 4;
int speed2 = 5;
int speed3 = 6;
int speed4 = 7;


void setup() {
  Serial.begin(9600);
  pinMode (cw1, OUTPUT);
  pinMode (ccw1, OUTPUT);
  pinMode (cw2, OUTPUT);
  pinMode (ccw2, OUTPUT);
  pinMode (cw3, OUTPUT);
  pinMode (ccw3, OUTPUT);
  pinMode (cw4, OUTPUT);
  pinMode (ccw4, OUTPUT);
  pinMode (speed1, OUTPUT);
  pinMode (speed2, OUTPUT);
  pinMode (speed3, OUTPUT);
  pinMode (speed4, OUTPUT);

}

void loop() {
  if (Serial.available() > 0) {
    char data = Serial.read();

    if ( data == 'f' ) {
      forward();
    }
    else if ( data == 's' ) {
      stopmotor();
    }
    else if ( data == 'b' ) {
      backward();
    }
  }
  Serial.flush();
}

void forward() {
  digitalWrite(cw1, HIGH);
  digitalWrite(ccw1, LOW);
  digitalWrite(cw2, HIGH);
  digitalWrite(ccw2, LOW);
  digitalWrite(cw3, HIGH);
  digitalWrite(ccw3, LOW);
  digitalWrite(cw4, HIGH);
  digitalWrite(ccw4, LOW);
  for (int v = 0; v <= 120; v++) {
    analogWrite(speed1, v);
    analogWrite(speed2, v);
    analogWrite(speed3, v);
    analogWrite(speed4, v);
  }

}

void stopmotor() {
  digitalWrite(cw1, LOW);
  digitalWrite(ccw1, LOW);
  digitalWrite(cw2, LOW);
  digitalWrite(ccw2, LOW);
  digitalWrite(cw3, LOW);
  digitalWrite(ccw3, LOW);
  digitalWrite(cw4, LOW);
  digitalWrite(ccw4, LOW);
  analogWrite(speed1, 0);
  analogWrite(speed2, 0);
  analogWrite(speed3, 0);
  analogWrite(speed4, 0);
}

void backward() {
  digitalWrite(cw1, LOW);
  digitalWrite(ccw1, HIGH);
  digitalWrite(cw2, LOW);
  digitalWrite(ccw2, HIGH);
  digitalWrite(cw3, LOW);
  digitalWrite(ccw3, HIGH);
  digitalWrite(cw4, LOW);
  digitalWrite(ccw4, HIGH);
  for (int v = 0; v <= 120; v++) {
    analogWrite(speed1, v);
    analogWrite(speed2, v);
    analogWrite(speed3, v);
    analogWrite(speed4, v);
  }
}
