#include "analogPH.h"
#define alp     1.00f
float EMA_function(float alpha, float latest, float stored){
  return roundF(alpha*latest,6) + roundF((1-alpha)*stored,6);
}
/* StabilityDetector Class */
StabilityDetector::StabilityDetector(float precision)
{
	this->stable = false;
	this->stableCount = 0;
	this->index = 0;
	for (size_t i = 0; i < arrayLength(valueBuffer); i++)
		this->valueBuffer[i] = 0.00f;
	this->precision = precision;
}

void StabilityDetector::updateValue(float value) {
	valueBuffer[index] = value;
	if(++index == 10) index = 0;
	if( getDeviasion(valueBuffer) <= precision ) stableCount++;
	else stableCount = 0;
	stable = (stableCount > 20);
	if (stable) stableCount = 20;
    Serial.println("==================== deviasion : "+String(getDeviasion(valueBuffer),5)+"\t stable count : "+String(stableCount));
}

void StabilityDetector::reset(void)
{
    this->stable = false;
	this->stableCount = 0;
	this->index = 0;
	for (size_t i = 0; i < arrayLength(valueBuffer); i++)
		this->valueBuffer[i] = 0.00f;
}

// constructor 
analogPH::analogPH(TwoWire  &line, uint8_t addr, float volt_low, float volt_mid)
{
    this->line = &line;
    this->calibrationVal.point = 2;
    this->calibrationVal.volt_low   = volt_low;
    this->calibrationVal.volt_mid   = volt_mid;
    this->calibrationVal.volt_high  = sqrt(-1);
    this->defaultLow = volt_low;
    this->defaultMid = volt_mid;
    ADCLine.init(line, ADCLine.L, ADCLine.L); 
    this->device_address = ADCLine.getAddress();
}

bool analogPH::begin()
{
    this->time_run = millis();
    line->beginTransmission(device_address);
    if(line->endTransmission() ==0) 
    {
        this->resetParam();
        return true ;
    }
    else                            return false;
}

analogPH & analogPH::resetParam(void)
{
    this->calibrationVal={this->defaultLow, this->defaultMid, sqrt(-1), 2};
    this->bufferCalibrationTmp={sqrt(-1),sqrt(-1),sqrt(-1),0};
    this->phStableCount.reset();
    this->pHVoltStable.reset();
    this->pHVoltStable.setPrecision(0.000075f);
    return *this;
}

analogPH&analogPH::initValue(pH_calibration_buffer calibrationBuffer)
{
    this->calibrationVal = calibrationBuffer;
    this->time_run = millis();
    return *this;
}

float analogPH::getPHSignal(void)
{
    ADCLine.selectChannel(ADCLine.CHANNEL_1, ADCLine.GAIN_1, ADCLine.MODE_UNIPOLAR);
    delayMicroseconds(100);
    //voltage.ph_signal = EMA_function(alp,(float)ADCLine.readADC(),voltage.ph_signal);
    this->voltage.ph_signal = roundF((float)ADCLine.readADC(),5);
    return this->voltage.ph_signal;
}

float analogPH::getRefVoltage(void)
{
    ADCLine.selectChannel(ADCLine.CHANNEL_0, ADCLine.GAIN_1, ADCLine.MODE_UNIPOLAR);
    delayMicroseconds(100);
    //voltage.reference = EMA_function(alp,(float)ADCLine.readADC(),voltage.reference);
    this->voltage.reference = roundF((float)ADCLine.readADC(),5); 
    return this->voltage.reference;
}

phADC analogPH::readVoltage()
{
    //float tmpVal = ;
    //EMA_function(alp,tmpVal, voltage.ph_predict_voltage);
    this->voltage.ph_predict_voltage = roundF((getPHSignal() - getRefVoltage())/2,5);
    return this->voltage;
}


analogPH & analogPH::singleReading(void)
{
    this->readVoltage();
    this->pHVoltStable.updateValue(this->voltage.ph_predict_voltage);
   // Serial.print("parameter\n low"+String(this->calibrationVal.volt_low,4));
   // Serial.print("\tmid "+String(this->calibrationVal.volt_mid,4));
   // Serial.print("\thigh "+String(this->calibrationVal.volt_high,4));
   // Serial.print("\tpgnd "+String(this->voltage.reference,4));
   // Serial.print("\tprb "+String(this->voltage.ph_signal,4));
   // Serial.println("\tv-ph "+String(this->voltage.ph_predict_voltage,4));
    if(this->time_run == 0)
        this->time_run = millis();
    if(millis() - time_run >= 15000UL)
    {
        this->temperature_setted = false;
        this->time_run = millis();
    }
    this->pH = 0;
      //readVoltage();
    if(this->calibrationVal.point > 2 && (!isnan(this->calibrationVal.volt_low)|| (this->calibrationVal.volt_low==0.00f)) && 
      ( !isnan(this->calibrationVal.volt_mid)  || (this->calibrationVal.volt_low==0.00f) ) && 
      ( !isnan(this->calibrationVal.volt_high) || (this->calibrationVal.volt_low==0.00f) ) )
    {
        //Serial.println("fungsi 3 point");
        if(this->voltage.ph_predict_voltage > this->calibrationVal.volt_mid)
            this->pH = 7 - 2.99f/ (this->calibrationVal.volt_low - this->calibrationVal.volt_mid) * (voltage.ph_predict_voltage - this->calibrationVal.volt_mid);
        else 
            this->pH = 7 - 2.99f/ (this->calibrationVal.volt_mid - this->calibrationVal.volt_high) * (voltage.ph_predict_voltage - this->calibrationVal.volt_mid);
    }
    else
    {
       // Serial.print("fungsi 2 point");
        if( isnan(this->calibrationVal.volt_low) || (this->calibrationVal.volt_low==0.00f) && !isnan(this->calibrationVal.volt_mid) ) 
        {
            this->pH = 7.00 - 2.99f/ (this->calibrationVal.volt_mid - this->calibrationVal.volt_high) * (this->voltage.ph_predict_voltage - this->calibrationVal.volt_mid);
         //   Serial.println(" a");
        }
            
        if  ( isnan(this->calibrationVal.volt_high) || (this->calibrationVal.volt_high==0.00f) && !isnan(this->calibrationVal.volt_mid) ) 
        {
            this->pH =   7 - 2.99f / (this->calibrationVal.volt_low - this->calibrationVal.volt_mid) * (this->voltage.ph_predict_voltage - this->calibrationVal.volt_mid);
          //  Serial.println(" b ph="+String(this->pH));
        }
             
    }
   // float compensation = ( (this->temperature+273.15) / (25+273.15) );
    //      compensation =  isnan(compensation)?1:(compensation<0)?0.001:compensation;
    //this->pH = 7.0 +  (( this->pH - 7.0 ) * compensation);

    

    this->pH = roundF(this->pH, 3);
    this->phStableCount.updateValue(this->pH);
    
    //this->pH = (this->pH > 14)?14:(this->pH<0)?0:this->pH;
    return *this;
}

bool analogPH::calibration(uint8_t level_ph)
{
    readVoltage();
    if(!this->pHVoltStable.isStable()) return false;
    if(cal_level_buffer != level_ph || level_ph==MID_PH)
    {
        switch(level_ph)
        {
            case LOW_PH  :
                            if(this->bufferCalibrationTmp.point <=1) return false;
                            if(this->temperature_setted)
                                voltage.ph_predict_voltage = (voltage.ph_predict_voltage - reverse_funct(4.01, temperature) ) + voltage.ph_predict_voltage; 
                            else
                                voltage.ph_predict_voltage = (voltage.ph_predict_voltage - reverse_funct(4.01, 25.00) ) + voltage.ph_predict_voltage;
                            
                            bufferCalibrationTmp.volt_low = voltage.ph_predict_voltage;
                            bufferCalibrationTmp.point++;
                            break;
            case MID_PH  :
                            bufferCalibrationTmp={sqrt(-1),sqrt(-1),sqrt(-1),0};
                            if(this->temperature_setted)
                                voltage.ph_predict_voltage = (voltage.ph_predict_voltage - reverse_funct(7.00, temperature) ) + voltage.ph_predict_voltage; 
                            else
                                voltage.ph_predict_voltage = (this->voltage.ph_predict_voltage - reverse_funct(7.00, 25.00) ) + this->voltage.ph_predict_voltage;
                            
                            bufferCalibrationTmp.volt_mid = voltage.ph_predict_voltage;
                            bufferCalibrationTmp.point=1;
                            calibrationVal.volt_mid = bufferCalibrationTmp.volt_mid;
                            calibrationVal.volt_low = calibrationVal.volt_low + (calibrationVal.volt_mid-defaultMid);
                            Serial.println("set mid point");
                            return true;
                            break;
            case HIGH_PH : 
                            if(this->bufferCalibrationTmp.point <=1) return false;
                            if(this->temperature_setted)
                                voltage.ph_predict_voltage = (voltage.ph_predict_voltage - reverse_funct(10.01, temperature) ) + voltage.ph_predict_voltage; 
                            else
                                voltage.ph_predict_voltage = (voltage.ph_predict_voltage - reverse_funct(10.00, 25.00) ) + voltage.ph_predict_voltage;
                            
                            bufferCalibrationTmp.volt_high = voltage.ph_predict_voltage;
                            bufferCalibrationTmp.point++;
                            break;
            default : return false;
        }
        cal_level_buffer = level_ph;
    };
    this->bufferCalibrationTmp.point = bufferCalibrationTmp.point>calibrationPointMAX?calibrationPointMAX:bufferCalibrationTmp.point;
    if(bufferCalibrationTmp.point > 1)
    {
        this->calibrationVal = bufferCalibrationTmp;
        return true;
    };
}

float analogPH::reverse_funct(float pH_, float temp_)
{
    Serial.println("pH_ = "+String(pH_)+"\t temp = "+String(temp_));
    float predict_voltage = 0;
    float predict_compensation = ( (temp_+273.15) / (25+273.15) );
    float tempPH = ( (pH_ - 7.00) / predict_compensation ) + 7.00;
    predict_voltage = (((tempPH-7.00)* (this->calibrationVal.volt_low - this->calibrationVal.volt_mid))-( 2.99 *this->calibrationVal.volt_mid)) / (-2.99);
    Serial.println("predic v "+String(predict_voltage));
    return roundF(predict_voltage,5);
}