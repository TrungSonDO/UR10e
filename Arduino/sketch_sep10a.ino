const int PUL = 3;   // STEP pin
const int DIR = 4;   // DIR pin

// Motor parameters
const int stepsPerRev = 200;   // motor steps per revolution (e.g. 200 for 1.8°)
const int microstepping = 1;   // set this according to your driver setting
const int totalSteps = stepsPerRev * microstepping;

float f_rev = 1;   // desired revolution frequency (Hz)

void waitMicros(unsigned long us) {
  if (us >= 1000) {
    delay(us / 1000);
    us = us % 1000;
  }
  if (us > 0) delayMicroseconds(us);
}

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, LOW);   // direction
  Serial.begin(9600);

  Serial.print("Running at freq: "); Serial.print(f_rev); Serial.println(" Hz");

  float omega = 2 * 3.14159 * f_rev;   // rad/s
  long f_pulse = f_rev * totalSteps;   // required pulse frequency
  unsigned long halfPeriod = 1000000UL / (2 * f_pulse);

  Serial.print("Omega: "); Serial.print(omega); Serial.print(" rad/s, ");
  Serial.print("Pulse freq: "); Serial.print(f_pulse); Serial.println(" Hz");

  // chạy trong 40 giây
  unsigned long t0 = millis();
  while (millis() - t0 < 40000) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5); // pulse width
    digitalWrite(PUL, LOW);
    waitMicros(halfPeriod - 5);
  }

  Serial.println("Finished 40 seconds run.");
}

void loop() {
  // không dùng
}
