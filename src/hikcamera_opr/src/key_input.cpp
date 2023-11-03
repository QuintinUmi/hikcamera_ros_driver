#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>

#include "kbhit.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "key_input");
    ros::NodeHandle rosHandle;

    ros::Publisher key_ascii_pub = rosHandle.advertise<std_msgs::Int8>("/msg_hikcamera/key_input", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        if(kbhit()){
            // printf("\n----------------------------------------------------------------------------------\n");
            char c = fgetc(stdin);
            std_msgs::Int8 ascii;
            ascii.data = int(c);
            key_ascii_pub.publish(ascii);
        }
        loop_rate.sleep();
    }

    return 0;
}
