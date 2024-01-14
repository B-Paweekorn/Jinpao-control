#include "QEI.h"

/*-----QEI(encA, encB)------*/

QEI enc0(18, 17); 
QEI enc1(33, 34);
QEI enc2(36, 35);
QEI enc3(37, 38);

long counter0 = 0;
long counter1 = 0;
long counter2 = 0;
long counter3 = 0;

void setup() {
  Serial.begin(115200);

  /*-----Setup QEI Start-----*/
  // pinMode(33,INPUT_PULLUP);
  // pinMode(34,INPUT_PULLUP);

  enc0.begin();
  enc1.begin();
  enc2.begin();
  enc3.begin();

  /*-----Setup QEI End-------*/
}

void loop() {
  uint32_t time = micros();

  counter0 += enc0.get_diff_count();
  counter1 += enc1.get_diff_count();
  counter2 += enc2.get_diff_count();
  counter3 += enc3.get_diff_count();

  uint32_t dt = micros() - time;

  // Serial.print(dt);
  // Serial.print(' ');
  Serial.print(counter0);
  Serial.print(' ');
  Serial.print(counter1);
  Serial.print(' ');
  Serial.print(counter2);
  Serial.print(' ');
  Serial.println(counter3);
}
