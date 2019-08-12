#ifndef analogPH_h
#define analogPH_h

#include <Arduino.h>
#include <Wire.h>
#include <Stream.h>
#include "support\filter\statistic.h"
#include "support\ADS1112\ADS1112.h"

#define calibrationPointMAX         3
#define LOW_PH                      4
#define MID_PH                      7
#define HIGH_PH                     10



struct phADC
{
    float reference;        
    float ph_signal;
    float ph_predict_voltage;
};

struct pH_calibration_buffer
{
    float volt_low,         // voltage at pH 4
          volt_mid,         // voltage at ph 7
          volt_high;        // voltage at ph 8
    uint8_t point;
};

class StabilityDetector 
{
private:
	bool stable;
	unsigned char stableCount;
	float precision=0.0125;
	unsigned char index;
	float valueBuffer[10];

public:
	StabilityDetector (float precision=0.005f);
	void setPrecision(float precision) { this->precision = precision; };
	void updateValue(float value);
	bool isStable(void) { return stable; };
	unsigned char getStableCount(void) { return stableCount; };
	float getModeValue(void) { return getMode(valueBuffer); };
	float getMedianValue(void) { return getMedian(valueBuffer); };
    void reset(void);
};

class analogPH:public ADS1112
{
    protected:
        float reverse_funct(float pH_, float temp_);
    private:
        uint8_t device_address;
        pH_calibration_buffer calibrationVal;
        pH_calibration_buffer bufferCalibrationTmp={sqrt(-1),sqrt(-1),sqrt(-1),0};
        phADC voltage;
        StabilityDetector phStableCount;
        StabilityDetector pHVoltStable;
        TwoWire *line;
        ADS1112 ADCLine;
        uint8_t cal_level_buffer = 0;
        float pH;
        float temperature = 25.00f; 
        float reverse_factor;
        bool temperature_setted = false;
        unsigned long time_run = 0;
        float defaultLow, defaultMid;

    public:
        analogPH(TwoWire &line, uint8_t addr, float volt_low = 0.518f, float volt_mid = 0.353f);
        bool begin();
        analogPH&resetParam(void);
        analogPH&initValue(pH_calibration_buffer calibrationBuffer);
        analogPH&setCalibrationPoint(uint8_t level, float volt);     
        phADC readVoltage();
        analogPH&singleReading(void);
        bool calibration(uint8_t level_ph);
        void setPrecision(float prec){this->phStableCount.setPrecision(prec);};
        void setTemperatureCompensation(float temp){this->temperature= temp; this->temperature_setted = true;};

        float getRefVoltage(void);
        float getPHSignal(void);
        float getPH(void){return this->pH;};

        bool isStable(void){return this->phStableCount.isStable();};
        unsigned char stableCount(void){return this->phStableCount.getStableCount();};
        //void setPrecision(float prec){this->phStableCount.setPrecision(prec);};
};
#endif