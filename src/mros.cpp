#include "mros.h"

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

//------------------------------------------------------------------------------
// Globals
//------------------------------------------------------------------------------

static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;
static rcl_timer_t     timer;

// Publishers
static rcl_publisher_t int_publisher;
static rcl_publisher_t string_publisher;

// Messages
static std_msgs__msg__Int32  int_msg;
static std_msgs__msg__String string_msg;

// Subscriber
static rcl_subscription_t        cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;

// CmdVel storage
static float cmd_vel_lin_x = 0.0f;
static float cmd_vel_lin_y = 0.0f;
static float cmd_vel_ang_z = 0.0f;

//------------------------------------------------------------------------------
// Error handling
//------------------------------------------------------------------------------

static void error_loop() {
    while (true) {
        delay(100);
    }
}

#define RCCHECK(fn)           \
    do {                      \
        rcl_ret_t rc = (fn);  \
        if (rc != RCL_RET_OK) \
            error_loop();     \
    } while (0)

#define RCSOFTCHECK(fn)      \
    do {                     \
        rcl_ret_t rc = (fn); \
        (void)rc;            \
    } while (0)

//------------------------------------------------------------------------------
// Timer callback
//------------------------------------------------------------------------------

static void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);

    if (timer == nullptr)
        return;

    RCSOFTCHECK(rcl_publish(&int_publisher, &int_msg, nullptr));
    int_msg.data++;
}

//------------------------------------------------------------------------------
// CmdVel callback
//------------------------------------------------------------------------------

static void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg =
        (const geometry_msgs__msg__Twist*)msgin;

    cmd_vel_lin_x = msg->linear.x;
    cmd_vel_lin_y = msg->linear.y;
    cmd_vel_ang_z = msg->angular.z;
}

//------------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------------

void mros_init(HardwareSerial& serial) {
    serial.begin(115200);
    set_microros_serial_transports(serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    RCCHECK(rclc_node_init_default(
        &node,
        "micro_ros_platformio_node",
        "",
        &support));

    //----------------------------------------------------------
    // Int32 publisher
    //----------------------------------------------------------

    RCCHECK(rclc_publisher_init_default(
        &int_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_platformio_node_publisher"));

    //----------------------------------------------------------
    // String publisher
    //----------------------------------------------------------

    RCCHECK(rclc_publisher_init_default(
        &string_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "status"));

    //----------------------------------------------------------
    // Initialize message memory
    //----------------------------------------------------------

    int_msg.data = 0;
    std_msgs__msg__String__init(&string_msg);

    //----------------------------------------------------------
    // CmdVel subscriber
    //----------------------------------------------------------

    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    //----------------------------------------------------------
    // Timer
    //----------------------------------------------------------

    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true));

    //----------------------------------------------------------
    // Executor
    //----------------------------------------------------------

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA));
}

void mros_spin() {
    RCSOFTCHECK(
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(10)));
}

void mros_publish_string(const char* text) {
    if (!rosidl_runtime_c__String__assign(&string_msg.data, text))
        return;

    RCSOFTCHECK(rcl_publish(&string_publisher, &string_msg, nullptr));
}

//------------------------------------------------------------------------------
// CmdVel getters
//------------------------------------------------------------------------------

float mros_get_cmd_vel_lin_x() { return cmd_vel_lin_x; }
float mros_get_cmd_vel_lin_y() { return cmd_vel_lin_y; }
float mros_get_cmd_vel_ang_z() { return cmd_vel_ang_z; }
