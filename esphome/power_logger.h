#include "esphome.h"
#include "SD.h"

#define LOGFILENAME "plogger.csv"

class powerLogger : public Component, public Sensor {
public:
    /**
     * The constructor of the powerLogger class receives references for the current
     * and voltage sensors created by the INA219 integration and uses their internal
     * values to efficiently do the calculations
     *
     * @param int_dt    Internal deltaT, The minimum timestep for pooling the measurements
     * @param inaCurr   Current sensor reference, received through the id(SENSOR_ID)
     * @param inaColt   Voltage sensor reference, received through the id(SENSOR_ID)
     * @param ina       Actual Component reference, received through the id(COMPONENT_ID)
     * 
     */
    powerLogger(uint16_t int_dt, Sensor *inaCurr, Sensor *inaVolt, PollingComponent *ina) : 
        int_dt(int_dt), inaCurrent(inaCurr), inaVoltage(inaVolt), inaComp(ina) {}
    
    float get_setup_priority() const override { return esphome::setup_priority::DATA; }
    
    Sensor *capacity = new Sensor();
    Sensor *energy = new Sensor();
    Sensor *sdLog = new Sensor();

    uint16_t int_dt;

    Sensor *inaCurrent = nullptr;
    Sensor *inaVoltage = nullptr;
    PollingComponent *inaComp = nullptr;

    const int chipSelect = D8;  // used for ESP8266
    bool sdCardPresent = true;

    unsigned long last_measurement = 0;
    float net_mah = 0.0f;    
    float net_wh = 0.0f;

    float last_c = 0.0f;
    float last_v = 0.0f;

    void setup() override {
        ESP_LOGD("custom", "Sending hello from plogger");
        inaComp->update();
        last_c = getCurr();
        last_v = getVolt();

        if(!SD.begin(chipSelect)){
            sdCardPresent = false;
            ESP_LOGE("custom","SDCARD Initialization failed!");
        } else if (!SD.exists(LOGFILENAME)){
            File dataFile = SD.open(LOGFILENAME, FILE_WRITE);
            if(dataFile){ 
                //Create the CSV header 
                dataFile.println("Voltage,Current,Power,mAh,mWh,Dt");
                dataFile.close();
            }
        }

        // Publish the SDCard presence so we can show it on the display
        sdLog->publish_state((int)sdCardPresent);
    }

    void loop() override {
        if(millis() > last_measurement + int_dt){
            inaComp->update();
            unsigned long dt = millis() - last_measurement;
            last_measurement += dt;
            float new_c = getCurr();
            float new_v = getVolt();
            
            //Integrate through the midpoint method
            float dmah = (((new_c - last_c)/2.0f)+last_c);
            if(dmah != 0.0f){ //Spare the calculations if value is zero
                dmah *= ((float)dt/3600.0f); //convert our current uAs to mAh           
                float dwh = (((new_v - last_v)/2.0f)+last_v)*dmah;
                
                net_mah += dmah;
                net_wh += dwh;
            }
            
            ESP_LOGVV("custom", "mAh: %.5f || mWh: %.5f || Dt: %lu", net_mah, net_wh, dt);
            capacity->publish_state(net_mah);
            energy->publish_state(net_wh);

            last_c = new_c;
            last_v = new_v;

            if(sdCardPresent){
                File dataFile = SD.open("plogger.txt", FILE_WRITE);
                if(dataFile){     
                    // Write a new csv line to the sdcard
                    //             Voltage(V),Current(A),Power(W),mAh,mWh,Dt
                    dataFile.printf("%.3f,%.5f,%.4f,%.3f,%.3f,%lu\n",new_v, new_c, new_v*new_c, net_mah, net_wh, dt);
                    dataFile.close();
                }else{
                    //if the file opening failed, disable sdcard altogether
                    sdCardPresent = false;
                    sdLog->publish_state((int)sdCardPresent);
                }
            }
        }
    }

    //Helper functions to get the raw sensor data and do some filtering beforehand
    float getCurr(){
        float value = inaCurrent->raw_state;
        if(fabs(value) < 0.0002) value = 0.0;
        return value;
    }

    float getVolt(){
        float value = inaVoltage->raw_state;
        return value;
    }

    // In case we would need to reset the values without reseting the whole node
    void resetCapacity(){
        net_mah = 0.0f;
    }

    void resetEnergy(){
        net_wh = 0.0f;
    }
};