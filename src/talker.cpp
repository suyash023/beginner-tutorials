/**
 *
 *  @file talker.cpp
 *  @date Oct 26, 2019
 *  @author Suyash Yeotikar
 *  @brief main file
 *  Copyright 2019 Suyash Yeotikar  [legal/copyright]
 *  @mainpage project page
 *  Please refer the talker.cpp file in file section
 *  and function members sections for detailed documentation
 */
#include <sstream>
#include <string>
#include "boost/date_time.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/change_string.h"



/**
 * @brief call back function for service to change the base string
 * @param req res the request and response that is received and sent by the service
 * @return bool true if the service was called and response was sent, false otherwise
 */
bool ChangeString( beginner_tutorials::change_string::Request &req,
         beginner_tutorials::change_string::Response &res) {
    std::string inputToChange = req.input;
    std::stringstream ss;
    boost::posix_time::ptime timeLocal = boost::posix_time::second_clock::
                                           local_time();
    if ( inputToChange.size() > 0 ) {
        ROS_INFO_STREAM("The input string to service was: " << inputToChange);
        ss << inputToChange << "on " << timeLocal;
        ROS_INFO_STREAM("Responding with date and time: " << ss.str());
        res.output = ss.str();
        ROS_DEBUG_STREAM("The string size was: " << inputToChange.size());
        return true;
    } else {
        ROS_FATAL_STREAM("The request string was empty!");
        ROS_DEBUG_STREAM("The string size was 0");
        return false;
    }
}
/**
 * @brief This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer server = n.advertiseService("change_string", ChangeString);
  ROS_INFO("Modified string");
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Hello to everyone in ENPM 808X! " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

