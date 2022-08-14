#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "pico.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


float clockDiv = 64;
float wrap = 39062;

// Set the servo to a specific angle
void setMillis(int servoPin, float millis)
{
    pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

// Initializes the servo and sets it into original position
void setServo(int servoPin, float startMillis)
{
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(5);
    clockDiv = 64;
    wrap = 39062;

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}


const uint LED_PIN = 25;
const int RIGHT_ARM_PIN = 22;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

// Changes bicep angle each call (called when a new message is posted to 'right_arm' topic)
void subscription_callback(const void * msgin){

    // Get message from 'right_arm' topic
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    
    // gpio_put(25,1);
    // sleep_ms(msg->data);
    // gpio_put(25,0);
    // sleep_ms(msg->data);


    float bicep_angle = msg->data;
    // Set the servo to the correct position/angle
    float millis_input = bicep_angle * 10.56;
    millis_input += 600;
    setServo(RIGHT_ARM_PIN, millis_input);

}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);


    // Node initialized
    rclc_node_init_default(&node, "pico_node_modified", "", &support);
    
    rclc_subscription_init_default(
        &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_arm"
    );


    // Executor initialized
    rclc_executor_init(&executor, &support.context, 1, &allocator);
        
    rclc_executor_add_subscription(
        &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA
    );
    

    // Sets the servo to 0 degrees
    setServo(RIGHT_ARM_PIN, 600);
    

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    }
    return 0;
}
