#include <stdlib.h>

#include "flow_AS5048B.h"
#include "temperature.h"
#include "Marlin.h"
#include "stepper.h"

#define MAX_MILLIS_FLOW_POSITION      (2000UL)
#define MIN_DELTA_FLOW_POSITION       (21)
#define MIN_DELTA_ANGLE               (11)
#define MAX_COUNT_FILAMENT_FLOW_ERROR (2)

struct flow_sensor_data
{
    bool          is_first_check_flow_position;
    bool          is_first_flow_position;
    bool          changing_flow_position;
    uint16_t      count_filament_flow_error;
    int16_t       angle;
    long          flow_position;
    unsigned long previous_millis;
};

static struct flow_sensor_data sd[NR_OF_FLOW_SENSORS];

void initFlowPosition(uint8_t n)
{
//    SERIAL_ECHOPGM("initFlowPosition, sensor=");
//    MSerial.println(n, HEX);

    sd[n].is_first_check_flow_position = true;
    sd[n].is_first_flow_position       = true;
    sd[n].changing_flow_position       = false;
    sd[n].count_filament_flow_error    = 0;
    sd[n].angle                        = 0;
    sd[n].flow_position                = 0;
    sd[n].previous_millis              = 0;
}

static int16_t readAngleFlowSensor(uint8_t n)
{
    uint16_t value;

    flowAS5048BStart(n);

    while (!flowAS5048BDone(n, value))
    {
        manage_heater();
        manage_inactivity();
    }

    return (int16_t) value;
}

static int checkFlowSensor(uint8_t n, bool flow_position_change_started)
{
    int16_t old_angle;

    if (flow_position_change_started)
    {
        sd[n].angle                     = readAngleFlowSensor(n);
        old_angle                       = sd[n].angle - MIN_DELTA_ANGLE;
        sd[n].count_filament_flow_error = 0;
    }
    else
    {
        old_angle   = sd[n].angle;
        sd[n].angle = readAngleFlowSensor(n);
    }
//    SERIAL_ECHOPGM("angle[");
//    MSerial.print(n, HEX);
//    SERIAL_ECHOPGM("]=  ");
//    MSerial.println(sd[n].angle);
//    SERIAL_ECHOPGM("old_angle=");
//    MSerial.println(old_angle);
    if (abs(sd[n].angle - old_angle) < MIN_DELTA_ANGLE)
    {
        sd[n].count_filament_flow_error++;
        if (sd[n].count_filament_flow_error == MAX_COUNT_FILAMENT_FLOW_ERROR)
        {
            SERIAL_ECHOLNPGM("ERROR: FILAMENT FLOW");
            sd[n].count_filament_flow_error = 0;
        }
    }
    else
    {
        sd[n].count_filament_flow_error = 0;
    }
}

void checkFlowPosition(uint8_t n)
{
    long old_flow_position;

    if (sd[n].is_first_check_flow_position)
    {
        sd[n].previous_millis              = millis();
        sd[n].is_first_check_flow_position = false;
//        SERIAL_ECHOPGM("sd[");
//        MSerial.print(n, HEX);
//        SERIAL_ECHOPGM("].previous_millis=");
//        MSerial.println(sd[n].previous_millis);
    }
    if (millis() - sd[n].previous_millis > MAX_MILLIS_FLOW_POSITION)
    {
//        SERIAL_ECHOPGM("millis()=");
//        MSerial.println(millis());
//        SERIAL_ECHOPGM("sd[");
//        MSerial.print(n, HEX);
//        SERIAL_ECHOPGM("].previous_millis=");
//        MSerial.println(sd[n].previous_millis);
        if (sd[n].is_first_flow_position)
        {
            sd[n].flow_position          = st_get_position(E_AXIS);
            old_flow_position            = sd[n].flow_position;
            sd[n].is_first_flow_position = false;
        }
        else
        {
            old_flow_position   = sd[n].flow_position;
            sd[n].flow_position = st_get_position(E_AXIS);
        }
//        SERIAL_ECHOPGM("flow position[");
//        MSerial.print(n, HEX);
//        SERIAL_ECHOPGM("]= ");
//        MSerial.println(sd[n].flow_position);
//        SERIAL_ECHOPGM("old flow position=");
//        MSerial.println(old_flow_position);
        if (abs(sd[n].flow_position - old_flow_position) > MIN_DELTA_FLOW_POSITION)
        {
            checkFlowSensor(n, !sd[n].changing_flow_position);
            sd[n].changing_flow_position = true;
        }
        else
        {
            sd[n].changing_flow_position = false;
        }
        sd[n].previous_millis = millis();
    }
}

