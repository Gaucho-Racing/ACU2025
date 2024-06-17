#ifndef BATTERY_H
#define BATTERY_H

class Battery
{
private:
    /* data */
public:
    Battery();
    float cellVoltage[140];
    float cellTemp[140];
    float balTemp[140]; 
    float cellSOC[140];
};

Battery::Battery()
{
}


#endif