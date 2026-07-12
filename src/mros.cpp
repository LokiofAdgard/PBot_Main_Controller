#include "mros.h"

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

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
static std_msgs__msg__String            debug_msg;
static std_msgs__msg__Float32MultiArray pc_msg;

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
    delay(1000);

    allocator = rcl_get_default_allocator();

    while (!mros_fail()) delay(1000);
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
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
    // CmdVel subscriber
    //----------------------------------------------------------

    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    //----------------------------------------------------------
    // PC Float32MultiArray publisher
    //----------------------------------------------------------

    std_msgs__msg__Float32MultiArray__init(&pc_msg);

    pc_msg.data.capacity = 18;
    pc_msg.data.size     = 18;
    pc_msg.data.data     = (float*)malloc(sizeof(float) * 18);

    RCCHECK(rclc_publisher_init_default(
        &pc_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
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

    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, nullptr));
}

void mros_publish_pc(const PC_t* pc) {
    float* d = pc_msg.data.data;

    d[0] = pc->solar.voltage;
    d[1] = pc->solar.current;
    d[2] = pc->solar.power;

    d[3] = pc->mppt.voltage;
    d[4] = pc->mppt.current;
    d[5] = pc->mppt.power;

    d[6] = pc->bat.voltage;
    d[7] = pc->bat.current;
    d[8] = pc->bat.power;

    d[9]  = pc->v5.voltage;
    d[10] = pc->v5.current;
    d[11] = pc->v5.power;

    d[12] = pc->v12a.voltage;
    d[13] = pc->v12a.current;
    d[14] = pc->v12a.power;

    d[15] = pc->v12b.voltage;
    d[16] = pc->v12b.current;
    d[17] = pc->v12b.power;

    RCSOFTCHECK(rcl_publish(&pc_publisher, &pc_msg, nullptr));
}

//------------------------------------------------------------------------------
// CmdVel getters
//------------------------------------------------------------------------------

float mros_get_cmd_vel_lin_x() { return cmd_vel_lin_x; }
float mros_get_cmd_vel_lin_y() { return cmd_vel_lin_y; }
float mros_get_cmd_vel_ang_z() { return cmd_vel_ang_z; }

bool mros_fail() { return (rmw_uros_ping_agent(100, 2) != RMW_RET_OK); }
