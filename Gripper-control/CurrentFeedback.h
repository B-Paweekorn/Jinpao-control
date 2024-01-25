#ifndef _CurrentFeedback_H__
#define _CurrentFeedback_H__

class CurrentFeedback{
  private:

  public:
    int adc0;
    int adc1;
    int adc0_offset;
    int adc1_offset;
    float adc0_Volt;
    float adc1_Volt;
    float current[2] = {0.0, 0.0};
    float voltage0_offset = 0.0;
    float voltage1_offset = 0.0;
    float sensitivity = 185.0;
    bool getOffset = false;

    float ads_sensitivity = 0.1875;

    int sda_pin;
    int scl_pin;

    CurrentFeedback(int sda, int scl);

    void begin();

    float *read_current();
};

#endif