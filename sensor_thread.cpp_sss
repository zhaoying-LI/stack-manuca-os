/**
 * @defgroup sensor_thread Sensor Thread
 * @{
 */

#include <string>
#include "threads.h"
#include "mbed_trace.h"
#include "rtos.h"
#include "global_params.h"
#include "conversions.h"
#include "persist_store.h"
#include "time_engine.h"
#include "trace_macro.h"
#include "trace_manager.h"
#include "tmp75.h"

#define TMP75_ADDR      0x4B

void execute_sensor_control(int& current_cycle_interval)
{
    #undef TRACE_GROUP
    #define TRACE_GROUP "SensorThread"

    /* Timeout set to 1 ms */ 
    osEvent evt = sensor_control_mail_box.get(1);
    if (evt.status == osEventMail)
    {
        sensor_control_mail_t *sensor_control_mail = (sensor_control_mail_t *)evt.value.p;
        std::string param = sensor_control_mail->param;
        free(sensor_control_mail->param);
        int value = sensor_control_mail->value;
        std::string msg_id = sensor_control_mail->msg_id;
        free(sensor_control_mail->msg_id);
        std::string endpoint_id = sensor_control_mail->endpoint_id;
        free(sensor_control_mail->endpoint_id);

        if ((param == "sensor_poll_rate") && (value >= 10))      // Lowest bound - 10 seconds
        {
            tr_info("Sensor poll rate changed to %d", value);
            DecadaServiceResponse(endpoint_id, msg_id, trace_name[POLL_RATE_UPDATE]);
            sensor_control_mail_box.free(sensor_control_mail);
            WriteCycleInterval(to_string(value*1000));          // Convert to miliseconds and save to persistence 
            current_cycle_interval = value*1000;
        }
        sensor_control_mail_box.free(sensor_control_mail);
    }

    return;
}

 /* [rtos: thread_3] SensorThread */
void sensor_thread(void)
{   
    #undef TRACE_GROUP
    #define TRACE_GROUP  "SensorThread"

    const int sensor_thread_sleep_ms = 1000;
        
    const PinName i2c_data_pin = PinName::PB_9;
    const PinName i2c_clk_pin = PinName::PB_6;

    Watchdog &watchdog = Watchdog::get_instance();

    int current_cycle_interval = StringToInt(ReadCycleInterval());
    int current_poll_count = current_cycle_interval / sensor_thread_sleep_ms;
    int poll_counter = 0;

    Tmp75 onboard_temp_sensor(i2c_data_pin, i2c_clk_pin);
    onboard_temp_sensor.Enable();

    while (1) 
    {
        /* Wait for MQTT connection to be up before continuing */
        event_flags.wait_all(FLAG_MQTT_OK, osWaitForever, false);

        current_poll_count = current_cycle_interval / sensor_thread_sleep_ms;

        if (poll_counter == 0)
        {
            /* Start of sensor data stream - Add header */
            llp_sensor_mail_t * llp_mail = llp_sensor_mail_box.calloc();
            while (llp_mail == NULL)
            {
                llp_mail = llp_sensor_mail_box.calloc();
                tr_warn("Memory full. NULL pointer allocated");
                ThisThread::sleep_for(500);
            }
            llp_mail->sensor_type = StringToChar("header");
            llp_mail->value = StringToChar("start");
            llp_mail->raw_time_stamp = RawRtcTimeNow();
            llp_sensor_mail_box.put(llp_mail);

            /* Read internal tmp75 */
            std::vector<std::pair<std::string, std::string>> s_data;
            int stat = onboard_temp_sensor.GetData(s_data);
            if (stat == SensorType::DATA_NOT_RDY || stat == SensorType::DATA_CRC_ERR)
            {
                tr_warn("Sensor data error");
            }
                
            if (stat == SensorType::DATA_OK)
            {   
                llp_mail = llp_sensor_mail_box.calloc();
                while (llp_mail == NULL)
                {
                    llp_mail = llp_sensor_mail_box.calloc();
                    tr_warn("Memory full. NULL pointer allocated");
                    ThisThread::sleep_for(500);
                }      
                llp_mail->sensor_type = StringToChar(s_data[0].first);
                llp_mail->value = StringToChar(s_data[0].second);
                llp_mail->raw_time_stamp = RawRtcTimeNow();
                llp_sensor_mail_box.put(llp_mail);
            }

            /* Poll other sensors here */

            /* End of sensor data stream  - Add footer */
            llp_mail = llp_sensor_mail_box.calloc();
            while (llp_mail == NULL)
            {
                llp_mail = llp_sensor_mail_box.calloc();
                tr_warn("Memory full. NULL pointer allocated");
                ThisThread::sleep_for(500);
            }
            llp_mail->sensor_type = StringToChar("header");
            llp_mail->value = StringToChar("end");
            llp_mail->raw_time_stamp = RawRtcTimeNow();
            llp_sensor_mail_box.put(llp_mail);
        }
        poll_counter++;

        if (poll_counter > current_poll_count)
        {
            poll_counter = 0;
        }

        execute_sensor_control(current_cycle_interval);
        
        watchdog.kick();

        ThisThread::sleep_for(sensor_thread_sleep_ms);
    }
}
 
 /** @}*/
