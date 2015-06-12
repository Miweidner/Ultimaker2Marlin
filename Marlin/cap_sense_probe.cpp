#include "Marlin.h"
#include "i2c_capacitance.h"
#include "cap_sense_probe.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"

static uint16_t samples[CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT];
uint8_t deviation_error = 0;

static uint16_t captureSample()
{
    uint16_t value;
    static uint16_t previous_value = 0;
    i2cCapacitanceStart();
    uint16_t reject_count = 0;
    while(reject_count < 50)
    {
        if (i2cCapacitanceDone(value))
        {
            if (value != previous_value)
            {
                previous_value = value;

                return value;
            }
            reject_count++;
            i2cCapacitanceStart();
            previous_value = value;
        }
        manage_heater();
        manage_inactivity();
    }
    // 50 samples have returned with the same value.
    // the sensor is not connected, return error and stop all motion.
    // to prevent the bed from colliding with the head.
    quickStop();
    return 0;
}

static uint16_t calculateAverage()
{
    uint32_t average = 0;
    for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
        average += samples[n];
    return average / CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT;
}

static uint16_t calculatedWeightedAverage()
{
    uint16_t average = calculateAverage();
    uint32_t standard_deviation = 0;
    for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
    {
        standard_deviation += (samples[n] - average) * (samples[n] - average);
    }
    standard_deviation /= CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT;
    standard_deviation = sqrt(standard_deviation);

    uint32_t weighted_average = 0;
    uint16_t count = 0;
    for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
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
    uint8_t found_zero_position = 0;

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
    int measurement_cycles = 0;
    int levelled_count = 0;
    float levelled_z_sum = 0;


    while(blocks_queued())
    {
        for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
        {
            samples[n] = captureSample();
            z_position += float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];
        }
        z_position /= CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT;

        uint16_t average = calculatedWeightedAverage();
        if (average == 0)
        {
            // the sensor is not connected, return error and stop all motion.
            // to prevent the bed from colliding with the head.
            quickStop(); // should already have been done.
            MSerial.println("ERROR: capacitive sensor not connected.");
        }

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

#ifdef BED_LEVELING_DEBUG
            MSerial.print("DEBUG: z_position: ");
            MSerial.println(z_position);
            MSerial.print("DEBUG: average: ");
            MSerial.println(average);
            MSerial.print("DEBUG: diff: ");
            MSerial.println(diff);
#endif /* BED_LEVELING_DEBUG */

            previous_sensor_average = average;
            if (noise_measure_delay > 0)
            {
                noise_measure_delay --;
            }
            else
            {
                if (found_zero_position == 0)
                {
                    if (diff * diff_average_count * 2 < diff_average)
                    {
                        quickStop();
                        found_zero_position++;

                        plan_buffer_line(x, y, current_position[Z_AXIS] + 2, current_position[E_AXIS], feedrate, active_extruder);
                    }
                }
                else
                {
                    if (diff < 0)
                    {
                        quickStop();

                        found_zero_position++;
                    }
                    measurement_cycles++;
                }
            }
        }
    }
    if (measurement_cycles > 3)
    {
        MSerial.print("ERROR:  measurement_cycles: ");
        MSerial.println(measurement_cycles);
    }
    if (found_zero_position != 2)
    {
        MSerial.print("ERROR: did not find zero pos ");
        MSerial.println(int(found_zero_position));
    }

    return (found_zero_position == 2 && measurement_cycles <= 3)?current_position[Z_AXIS]:200;
}

float probeWithCapacitiveSensor(float x, float y)
{
    int tests = CONFIG_BED_LEVEL_PROBE_REPEAT;
    float height_sum = 0;
    float height_average = 0;
    float start_height = 5;
    float feedrate = 0.4;
    float height_count;
    deviation_error = 0;

    for(int i = 1; i < tests+1; i++)
    {
        float tmp = probeWithCapacitiveSensorOnce(x,y,start_height, feedrate/i);
        if (tmp > 50)
            return tmp;
        height_sum += tmp*i;
        height_count += i;
        height_average = height_sum/height_count;
        start_height = height_average+0.7*((11-i)/10.0);
    }
    return height_average;
}
