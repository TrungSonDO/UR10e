const int PUL = 3;   // STEP pin
const int DIR = 4;   // DIR pin

// Motor parameters
const int stepsPerRev = 200;   // motor steps per revolution (e.g. 200 for 1.8°)
const int microstepping = 1;   // set this according to your driver setting
const int totalSteps = stepsPerRev * microstepping;

int freqs[] = {1,2,3,4,5,6,7,8,9,10}; // revolution frequencies (Hz)
int numFreqs = sizeof(freqs) / sizeof(freqs[0]);

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
}

void loop() {
  for (int i = 0; i < numFreqs; i++) {
    int f_rev = freqs[i];  // desired rev/s

    if (f_rev == 0) {
      Serial.println("Stopped");
      delay(1000);  // 1 second stop
    } else {
      float omega = 2 * 3.14159 * f_rev;  // rad/s
      long f_pulse = f_rev * totalSteps;  // required pulse frequency
      unsigned long halfPeriod = 1000000UL / (2 * f_pulse);

      Serial.print("Target freq: "); Serial.print(f_rev); Serial.print(" Hz, ");
      Serial.print("Omega: "); Serial.print(omega); Serial.print(" rad/s, ");
      Serial.print("Pulse freq: "); Serial.print(f_pulse); Serial.println(" Hz");

      unsigned long t0 = millis();
      while (millis() - t0 < 40000) {  // run 40 seconds at this speed
        digitalWrite(PUL, HIGH);
        delayMicroseconds(5); // pulse width
        digitalWrite(PUL, LOW);
        waitMicros(halfPeriod - 5);
      }

      // stop for 5 seconds before next frequency
      Serial.println("Pausing 5 seconds...");
      delay(5000);
    }
  }
}
