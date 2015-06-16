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
                if (value == 0xffff)
                    return 0;
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

    uint8_t noise_measure_delay = 6;
    int measurement_cycles = 0;
    int levelled_count = 0;
    float levelled_z_sum = 0;


    while(blocks_queued())
    {
        for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
        {
            samples[n] = captureSample();
            z_position += float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];
            if (samples[n] == 0)
            {
                // the sensor is not connected, return error and stop all motion.
                // to prevent the bed from colliding with the head.
                quickStop(); // should already have been done.
                MSerial.println("ERROR: capacitive sensor not connected.");
            }
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

            DEBUG_PRINT("z_position: ");
            DEBUG_PRINTLNV(z_position);
            DEBUG_PRINT("average: ");
            DEBUG_PRINTLNV(average);
            DEBUG_PRINT("diff: ");
            DEBUG_PRINTLNV(diff);

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

typedef enum
{
    Z_DOWN = 0,
    Z_UP = 1
} Direction;

int singleStepZ(float x, float y, Direction dir, uint16_t steps)
{
    DEBUG_PRINT("z_position: ");
    DEBUG_PRINTLNV(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS], 5);
    plan_buffer_step(0,0, (dir==Z_UP?-1l*long(steps):steps),0,1,active_extruder);

    while(blocks_queued())
    {
        // wait
        manage_heater();
        manage_inactivity();
    }

    DEBUG_PRINT("new z_position: ");
    DEBUG_PRINTLNV(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS], 5);

}

float singleStepProbeWithCapacitiveSensor(float x, float y)
{
    const int diff_average_count = 6;
    uint8_t found_zero_position = 0;

    uint16_t previous_sensor_average = 0;
    int16_t diff_average = 0;
    int16_t diff_history[diff_average_count];
    uint8_t diff_history_pos = 0;

    for(uint8_t n=0; n<diff_average_count; n++)
        diff_history[n] = 0;

    uint8_t noise_measure_delay = 6;
    int measurement_cycles = 0;
    int levelled_count = 0;
    float levelled_z_sum = 0;

    DEBUG_PRINTLN("going to singleStepZ 6 steps down");
    singleStepZ(x, y, Z_DOWN, 6);
    DEBUG_PRINTLN("done singleStepZ 6 steps down");

    for(int i = 0; i < 10000; i++)
    {
        for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
        {
            samples[n] = captureSample();
            if (samples[n] == 0)
                return 0;
        }

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

            DEBUG_PRINT("z_position: ");
            DEBUG_PRINTLNV(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS], 5);
            DEBUG_PRINT("found_zero_position: ");
            DEBUG_PRINTLNV(int(found_zero_position));
            DEBUG_PRINT("average: ");
            DEBUG_PRINTLNV(average);
            DEBUG_PRINT("diff: ");
            DEBUG_PRINTLNV(diff);
            DEBUG_PRINT("diffaverage: ");
            DEBUG_PRINTLNV(float(diff_average)/diff_average_count, 5);

            previous_sensor_average = average;
            if (noise_measure_delay > 0)
            {
                noise_measure_delay --;
                DEBUG_PRINTLN("going to singleStepZ 1 step up");
                singleStepZ(x, y, Z_UP, 1);
                DEBUG_PRINTLN("done singleStepZ 1 step up");
            }
            else
            {
                if (found_zero_position == 0)
                {
                    if (diff * diff_average_count * 2 < diff_average)
                        found_zero_position++;
                    else
                    {
                        DEBUG_PRINTLN("going to singleStepZ 1 step up");
                        singleStepZ(x, y, Z_UP, 1);
                        DEBUG_PRINTLN("done singleStepZ 1 step up");
                    }
                }
                else
                {
                    if (diff < 0)
                    {
                        found_zero_position++;
                        if (measurement_cycles > 3)
                        {
                            MSerial.print("ERROR:  measurement_cycles: ");
                            MSerial.println(measurement_cycles);
                        }
                        return float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];
                    }
                    else
                    {
                        DEBUG_PRINTLN("going to singleStepZ 1 step down");
                        singleStepZ(x, y, Z_DOWN, 1);
                        DEBUG_PRINTLN("done singleStepZ 1 step down");
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
    return 200;
}

float probeWithCapacitiveSensor(float x, float y)
{
    int tests = 1; //CONFIG_BED_LEVEL_PROBE_REPEAT;
    float height_sum = 0;
    float height_average = 0;
    float start_height = 5;
    float feedrate = 0.4;
    float height_count = 0;
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
    //return height_average;    
    return singleStepProbeWithCapacitiveSensor(x,y);
}

uint16_t getCapacitiveMeasurement()
{
    for(uint8_t n=0; n<CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT; n++)
    {
        samples[n] = captureSample();
        if (samples[n] == 0)
            return 0;
    }
    return calculatedWeightedAverage();
}

int capacitiveSensorWorks()
{
    uint16_t heights[] = { CONFIG_BED_LEVELING_SENSOR_TEST_HEIGHT_1,
                           CONFIG_BED_LEVELING_SENSOR_TEST_HEIGHT_2,
                           CONFIG_BED_LEVELING_SENSOR_TEST_HEIGHT_3 };
#define SIZE sizeof(heights)/sizeof(heights[0])
    uint16_t measurements[SIZE];
    uint16_t diff[SIZE-1];

    for (int i= 0; i < SIZE; i++)
    {
        // move bed to CONFIG_BED_LEVELING_SENSOR_TEST_HEIGHT_?
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], heights[i], current_position[E_AXIS], homing_feedrate[Z_AXIS], active_extruder);

        st_synchronize();
        int wait = 100 + millis();  // keep track of when we started waiting
        while(millis()  < wait ){
            manage_heater();
            manage_inactivity();
        }

        // get reading
        measurements[i] = getCapacitiveMeasurement();
        MSerial.print("measurements[");MSerial.print(i);MSerial.print("]: ");MSerial.println(measurements[i]);
        if (i > 0)
        {
            diff[i-1] = ( measurements[i] - measurements[i-1] );
            MSerial.print("diff[");MSerial.print(i-1);MSerial.print("]: ");MSerial.println(diff[i-1]);
        }
    }
    return ((diff[0]*2) < diff[1]);
}
