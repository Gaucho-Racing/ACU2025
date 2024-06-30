#ifndef BATTERY_H
#define BATTERY_H

class Battery
{
private:
    /* data */
    int errors;
public:
    Battery();
    bool checkIsAlive();
    bool systemChecks();
    float cellVoltage[140];
    float cellTemp[140];
    float balTemp[140]; 
    float cellSOC[140];
};

Battery::Battery()
{
}
bool Battery::checkIsAlive(){
    return true;
}
bool Battery::systemChecks(){
    return true;
}


#endif