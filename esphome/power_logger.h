#include "esphome.h"

class powerLogger : public Component, public Sensor{
public:
    powerLogger(uint16_t int_dt, Sensor *inaCurr, Sensor *inaVolt, PollingComponent *ina) : 
        int_dt(int_dt), inaCurrent(inaCurr), inaVoltage(inaVolt), inaComp(ina) {}
    
    float get_setup_priority() const override { return esphome::setup_priority::DATA; }
    
    Sensor *capacity = new Sensor();
    Sensor *energy = new Sensor();

    uint16_t int_dt;

    Sensor *inaCurrent = nullptr;
    Sensor *inaVoltage = nullptr;
    PollingComponent *inaComp = nullptr;

    unsigned long last_measurement = 0;
    float net_mah = 0.0f;
    float input_mah = 0.0f;
    float output_mah = 0.0f;
    
    float net_wh = 0.0f;
    float input_wh = 0.0f;
    float output_wh = 0.0f;

    float last_c = 0.0f;
    float last_v = 0.0f;

    void setup() override {
        // This will be called by App.setup()
        ESP_LOGW("custom", "This is from plog");
        inaComp->update();
        last_c = getCurr();
        last_v = getVolt();
    }
    void loop() override {
        if(millis() > last_measurement + int_dt){
            inaComp->update();
            unsigned long dt = millis() - last_measurement;
            last_measurement += dt;
            float new_c = getCurr();
            float new_v = getVolt();
            
            float dmah = (((new_c - last_c)/2.0f)+last_c);
            if(dmah != 0.0f){
                dmah /= (3600.0f/(float)dt);            
                float dwh = (((new_v - last_v)/2.0f)+last_v)*dmah;
                
                net_mah += dmah;
                net_wh += dwh;
                if(dmah > 0.0f){
                    input_mah+=dmah;
                    input_wh+=dwh;
                }else if(dmah < 0.0f){
                    output_mah+=dmah;
                    output_wh+=dwh;
                }
            }
            
            ESP_LOGVV("custom", "mAh: %.5f || mWh: %.5f || Dt: %lu", net_mah, net_wh, dt);
            capacity->publish_state(net_mah);
            energy->publish_state(net_wh);

            last_c = new_c;
            last_v = new_v;
        }
    }

    float getCurr(){
        float value = inaCurrent->raw_state;
        if(fabs(value) < 0.0002) value = 0.0;
        return value;
    }

    float getVolt(){
        float value = inaVoltage->raw_state;
        return value;
    }

    void resetCapacity(){
        net_mah = 0.0f;
        input_mah = 0.0f;
        output_mah = 0.0f;
    }

    void resetEnergy(){
        net_wh = 0.0f;
        input_wh = 0.0f;
        output_wh = 0.0f;
    }
};