#import "ROS2Bridge.h"

#import <rcl/rcl.h>
#include <rcl/node.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <std_msgs/msg/string.h>

@implementation ROS2Bridge
{
    rcl_node_t node;
    rcl_publisher_t publisher;
    std_msgs__msg__String msg;
}

- (void)startPublishing
{
    // Copied from
    // https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/int32_publisher/main.c
    printf("Start publishing");

    // Sample code from documentation of rcl_publisher_init of rcl/publisher.h
    // Unfortunately, it has wrong parameters for rcl_node_init so copied from rcl/node.h

    rcl_context_t context = rcl_get_zero_initialized_context();
    node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_ret_t ret = rcl_node_init(&node, "minimal_publisher", "/ros2example", &context, &node_ops);
    // ... error handling

    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    publisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    ret = rcl_publisher_init(&publisher, &node, ts, "chatter", &publisher_ops);
    // ... error handling, and on shutdown do finalization:

    ret = rcl_publish(&publisher, &msg, NULL);
}

- (void)stopPublishing
{
    rcl_ret_t ret = rcl_publisher_fini(&publisher, &node);
    // ... error handling for rcl_publisher_fini()
    ret = rcl_node_fini(&node);
    // ... error handling for rcl_deinitialize_node()
}

- (void)startListening
{
    printf("Start listening");
}

@end
