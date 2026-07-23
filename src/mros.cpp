#include "mros.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int16_multi_array.h>

//------------------------------------------------------------------------------
// Globals
//------------------------------------------------------------------------------

static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t debug_publisher;
static rcl_publisher_t pc_publisher;

// Messages
static std_msgs__msg__String           debug_msg;
static std_msgs__msg__UInt16MultiArray pc_msg;

// Subscriber
static rcl_subscription_t             cmd_vel_sub;
static std_msgs__msg__Int16MultiArray cmd_vel_msg;

// CmdVel storage
static int16_t motor_l = 0;
static int16_t motor_r = 0;

//------------------------------------------------------------------------------
// Error handling
//------------------------------------------------------------------------------

static void error_loop() {
    delay(1000);
    esp_restart();
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
// CmdVel callback
//------------------------------------------------------------------------------

static void cmd_vel_callback(const void* msgin) {
    const std_msgs__msg__Int16MultiArray* msg =
        (const std_msgs__msg__Int16MultiArray*)msgin;

    if (msg->data.size >= 2) {
        motor_l = msg->data.data[0];
        motor_r = msg->data.data[1];
    }
}

//------------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------------

void mros_init(HardwareSerial& serial) {
    serial.begin(115200);
    set_microros_serial_transports(serial);
    delay(1000);

    allocator = rcl_get_default_allocator();

    while (!mros_fail()) delay(1000);

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(
        &node,
        "esp_controller_node",
        "",
        &support));

    //----------------------------------------------------------
    // Debug String publisher
    //----------------------------------------------------------

    std_msgs__msg__String__init(&debug_msg);

    RCCHECK(rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "debug"));

    //----------------------------------------------------------
    // CmdVel subscriber (UInt16MultiArray[2])
    //----------------------------------------------------------

    std_msgs__msg__Int16MultiArray__init(&cmd_vel_msg);

    cmd_vel_msg.data.capacity = 2;
    cmd_vel_msg.data.size     = 2;
    cmd_vel_msg.data.data     = (int16_t*)malloc(sizeof(int16_t) * 2);

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "cmd_vel"));

    //----------------------------------------------------------
    // PC UInt16MultiArray publisher (18 values)
    //----------------------------------------------------------

    std_msgs__msg__UInt16MultiArray__init(&pc_msg);

    pc_msg.data.capacity = 19;
    pc_msg.data.size     = 19;
    pc_msg.data.data     = (uint16_t*)malloc(sizeof(uint16_t) * 19);

    RCCHECK(rclc_publisher_init_default(
        &pc_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "pc_bus"));

    //----------------------------------------------------------
    // Executor
    //----------------------------------------------------------

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA));
}

void mros_spin() {
    RCSOFTCHECK(rclc_executor_spin_some(
        &executor,
        RCL_MS_TO_NS(10)));
}

void mros_debug(const char* text) {
    if (!rosidl_runtime_c__String__assign(&debug_msg.data, text))
        return;

    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

void mros_publish_pc(const PC_t* pc) {
    uint16_t* d = pc_msg.data.data;

    d[0] = (uint16_t)(pc->solar.voltage);
    d[1] = (uint16_t)(pc->solar.current);
    d[2] = (uint16_t)(pc->solar.power);

    d[3] = (uint16_t)(pc->mppt.voltage);
    d[4] = (uint16_t)(pc->mppt.current);
    d[5] = (uint16_t)(pc->mppt.power);

    d[6] = (uint16_t)(pc->bat.voltage);
    d[7] = (uint16_t)(pc->bat.current);
    d[8] = (uint16_t)(pc->bat.power);

    d[9]  = (uint16_t)(pc->v5.voltage);
    d[10] = (uint16_t)(pc->v5.current);
    d[11] = (uint16_t)(pc->v5.power);

    d[12] = (uint16_t)(pc->v12a.voltage);
    d[13] = (uint16_t)(pc->v12a.current);
    d[14] = (uint16_t)(pc->v12a.power);

    d[15] = (uint16_t)(pc->v12b.voltage);
    d[16] = (uint16_t)(pc->v12b.current);
    d[17] = (uint16_t)(pc->v12b.power);

    d[18] = (uint16_t)(pc->temp);

    RCSOFTCHECK(rcl_publish(&pc_publisher, &pc_msg, NULL));
}

//------------------------------------------------------------------------------
// CmdVel getters
//------------------------------------------------------------------------------

int16_t mros_get_motor_l() { return motor_l; }
int16_t mros_get_motor_r() { return motor_r; }

bool mros_fail() {
    return (rmw_uros_ping_agent(100, 2) != RMW_RET_OK);
}
