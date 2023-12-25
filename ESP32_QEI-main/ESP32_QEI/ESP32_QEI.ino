#include "QEI.h"

/*-----QEI(encA, encB)------*/

QEI enc0(9, 10); 
QEI enc1(7, 8);
QEI enc2(4, 5);
QEI enc3(1, 2);

long counter0 = 0;
long counter1 = 0;
long counter2 = 0;
long counter3 = 0;

void setup() {
  Serial.begin(115200);

  /*-----Setup QEI Start-----*/

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

  Serial.print(dt);
  Serial.print(' ');
  Serial.print(counter0);
  Serial.print(' ');
  Serial.print(counter1);
  Serial.print(' ');
  Serial.print(counter2);
  Serial.print(' ');
  Serial.println(counter3);
}
