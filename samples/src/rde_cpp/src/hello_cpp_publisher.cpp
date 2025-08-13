#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "hello_cpp_publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node which keeps track of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will begin to receive messages.  The second parameter to advertise()
   * is the size of the message queue used for publishing messages.  If messages
   * are published more quickly than we can send them, the number here specifies how
   * many messages to buffer up before throwing some away.
   */
  ros::Publisher hello_pub = n.advertise<std_msgs::String>("hello_cpp", 1000);

  ros::Rate loop_rate(1); // 1 Hz

  ROS_INFO("Hello C++ Publisher node started. Publishing to /hello_cpp topic.");

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Hello C++! Message #" << count << " at time: " << ros::Time::now();
    msg.data = ss.str();

    ROS_INFO("Published: %s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    hello_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ROS_INFO("Hello C++ Publisher node stopped.");
  return 0;
}
