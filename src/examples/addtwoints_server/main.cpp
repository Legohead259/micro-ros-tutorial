// We include the Arduino library for general Arduino functionality
#include <Arduino.h>
// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "tutorial_interfaces/srv/add_three_ints.h"
#include "tutorial_interfaces/msg/custom_message.h"

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

tutorial_interfaces__srv__AddThreeInts_Response res;
tutorial_interfaces__srv__AddThreeInts_Request req;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_service_t service;
rclc_executor_t executor;

void service_callback(const void * req, void * res){
    tutorial_interfaces__srv__AddThreeInts_Request * req_in = (tutorial_interfaces__srv__AddThreeInts_Request *) req;
    tutorial_interfaces__srv__AddThreeInts_Response * res_in = (tutorial_interfaces__srv__AddThreeInts_Response *) res;

    printf("Service request value: %d + %d + %d.\n", (int) req_in->a, (int) req_in->b, (int) req_in->c);

    res_in->sum = req_in->a + req_in->b + req_in->c;
}

void setup(void) {
    // Start serial communication with a baud rate of 115200
    Serial.begin(115200);
    // Configure Micro-ROS library to use Arduino serial
    set_microros_serial_transports(Serial);
    // Allow some time for everything to start properly
    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "add_twoints_server_rclc", "", &support));

    // create service
    RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(tutorial_interfaces, srv, AddThreeInts), "/addthreeints"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
}

void loop(void) {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    // RCCHECK(rcl_service_fini(&service, &node));
    // RCCHECK(rcl_node_fini(&node));
}