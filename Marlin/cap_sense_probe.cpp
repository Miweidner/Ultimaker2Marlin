#include "Marlin.h"
#include "i2c_capacitance.h"
#include "cap_sense_probe.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"

static const int sample_count = 100;
static uint16_t samples[sample_count];
uint8_t deviation_error = 0;


#define MAX_LEVEL_COUNT 3

static uint16_t captureSample()
{
    uint16_t value;
    static uint16_t previous_value = 0;
    i2cCapacitanceStart();
    while(true)
    {
        if (i2cCapacitanceDone(value))
        {
            if (value != previous_value)
            {
                previous_value = value;

                return value;
            }
            i2cCapacitanceStart();
            previous_value = value;
        }
        manage_heater();
        manage_inactivity();
    }
}

static uint16_t calculateAverage()
{
    uint32_t average = 0;
    for(uint8_t n=0; n<sample_count; n++)
        average += samples[n];
    return average / sample_count;
}

static uint16_t calculatedWeightedAverage()
{
    uint16_t average = calculateAverage();
    uint32_t standard_deviation = 0;
    for(uint8_t n=0; n<sample_count; n++)
    {
        standard_deviation += (samples[n] - average) * (samples[n] - average);
    }
    standard_deviation /= sample_count;
    standard_deviation = sqrt(standard_deviation);

    uint32_t weighted_average = 0;
    uint16_t count = 0;
    for(uint8_t n=0; n<sample_count; n++)
    {
        if (abs(samples[n] - average) <= standard_deviation)
        {
            count += 1;
            weighted_average += samples[n];
        }
    }
    if (!deviation_error && standard_deviation*100.0/(weighted_average / float(count)) > 2)
    {
        MSerial.print("ERROR: bed leveling detects too much vibration, standard_deviation/avg*100: ");
        MSerial.println(standard_deviation*100.0/(weighted_average / float(count)), 5 );
        deviation_error++;
    }

    return weighted_average / count;
}

float probeWithCapacitiveSensorOnce(float x, float y, float z_distance, float feedrate)
{
    const int diff_average_count = 6;
    float z_target = 0.0;
    float z_position = 0.0;
    uint8_t foundZeroPosition = 0;

    plan_buffer_line(x, y, z_target + z_distance, current_position[E_AXIS], homing_feedrate[Z_AXIS], active_extruder);
    st_synchronize();
    plan_buffer_line(x, y, z_target - 3, current_position[E_AXIS], feedrate, active_extruder);

    uint16_t previous_sensor_average = 0;
    int16_t diff_average = 0;
    int16_t diff_history[diff_average_count];
    uint8_t diff_history_pos = 0;

    for(uint8_t n=0; n<diff_average_count; n++)
        diff_history[n] = 0;

    uint8_t noise_measure_delay = 5;
    int steps = 0;
    int levelled_count = 0;
    float levelled_z_sum = 0;


    while(blocks_queued())
    {
        for(uint8_t n=0; n<sample_count; n++)
        {
            samples[n] = captureSample();
            z_position += float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];
        }
        z_position /= sample_count;

        uint16_t average = calculatedWeightedAverage();

        if (previous_sensor_average == 0)
        {
            previous_sensor_average = average;
        }
        else
        {
            int16_t diff = average - previous_sensor_average;

            diff_average -= diff_history[diff_history_pos];
            diff_average += diff;
            diff_history[diff_history_pos] = diff;
            diff_history_pos ++;
            if (diff_history_pos >= diff_average_count)
                diff_history_pos = 0;

            previous_sensor_average = average;
            if (noise_measure_delay > 0)
            {
                noise_measure_delay --;
            }
            else
            {
                if (foundZeroPosition == 0)
                {
                    if (diff * diff_average_count * 2 < diff_average)
                    {
                        quickStop();
                        foundZeroPosition++;

                        plan_buffer_line(x, y, current_position[Z_AXIS]+2, current_position[E_AXIS], feedrate, active_extruder);
                    }
                }
                else
                {
                    if (diff < 0)
                    {
                        quickStop();

                        foundZeroPosition++;
                    }
                    steps++;
                }
            }
        }
    }
    if (steps > 3)
    {
        MSerial.print("ERROR:  steps: ");
        MSerial.println(steps);
    }
    if (foundZeroPosition != 2)
    {
        MSerial.print("ERROR: did not find zero pos ");
        MSerial.println(int(foundZeroPosition));
    }

    return (foundZeroPosition == 2 && steps <= 3)?current_position[Z_AXIS]:200;
}

float probeWithCapacitiveSensor(float x, float y)
{
    int tests = MAX_LEVEL_COUNT;
    float height_sum = 0;
    float height_average = 0;
    float startHeight = 3;
    float feedrate = 0.4;
    float height_count;
    deviation_error = 0;

    for(int i = 1; i < tests+1; i++)
    {
        float tmp = probeWithCapacitiveSensorOnce(x,y,startHeight, feedrate/i);
        if (tmp > 50)
            return tmp;
        height_sum += tmp*i;
        height_count += i;
        height_average = height_sum/height_count;
        startHeight = height_average+0.7*((11-i)/10.0);
    }
    return height_average;
}
