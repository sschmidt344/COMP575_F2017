#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h>

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;

// ------------------------------------------------------------------------------------------------

class Rover {
private:
    std::string roverName;
    pose location;

public:
    Rover (std::string name, double x, double y, double theta) {
        roverName = name;
        setPose(x, y, theta);
    }

    Rover (std::string name, pose pos);

    void setPose(pose po) {
        setPose(po.x, po.y, po.theta);
    }

    void setPose(double x, double y, double theta) {
        location.x = x;
        location.y = y;
        location.theta = theta;
    }

    string getName() { return roverName; }
    double getX() { return location.x; }
    double getY() { return location.y; }
    double getTheta() { return location.theta; }
    pose getPose() { return location; }

};

std::vector<Rover> roverList;

double calculateGlobalAverageHeading() {
    std::vector<doubles> headingList;

    for (int i = 0; i < roverList.size(); i++) {
        headingList.emplace_back(roverList[i].getTheta());
    }

    return (double) accumulate(headingList.begin(), headingList.end(), 0.0) / headingList.size();
}

std::vector<unsigned int> calculateLocalNeighbors(Rover currentRover) {
    std::vector<unsigned int> localNeighbors;
    double distance;

    for (int i = 0; i < roverList.size(); i++) {
        // if not the same rover, calculate if the rover is a local neighbor
        if (!currentRover.getName().equals(roverList[i].getName())) {
            // use distance equation here between current rover and rover from list
            distance = sqrt((curentRover.getX() - roverList[i].getX())^2 + (currentRover.getY() - roverList[i].getY())^2);

            if (distance < 2) {
                localNeighbors.emplace_back(i);
            }
        }
    }
    return localNeighbors;
}

double calculateLocalAverageHeading(Rover currentRover) {
    std::vector<unsigned int> localNeighbors = calculateLocalNeighbors(currentRover);

    std::vector<doubles> headingList;

    for (int i = 0; i < localNeighbors.size(); i++) {
        headingList.emplace_back(roverList[localNeighbors].getTheta());
    }

    return (double) accumulate(headingList.begin(), headingList.end(), 0.0) / headingList.size();
}


// ------------------------------------------------------------------------------------------------

string rover_name;
char host[128];
bool is_published_name = false;


int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
ros::Publisher posePublish;
ros::Publisher global_average_heading_publisher;
ros::Publisher local_average_heading_publisher;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;

ros::Subscriber messageSubscriber;
ros::Subscriber poseSubscriber;
ros::Subscriber global_average_heading_subscriber;
ros::Subscriber local_average_heading_subscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); // 
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);
void poseHandler(const std_msgs::String::ConstPtr &message);
void global_average_heading_handler(const std_msgs::String::ConstPtr &message);
void local_average_heading_handler(const std_msgs::String::ConstPtr &message);

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
    poseSubscriber = mNH.subscribe(("poses"), 10, poseHandler);
    global_average_heading_subscriber = mNH.subscribe("global average heading", 10, global_average_heading_publisher);
    local_average_heading_subscriber = mNH.subscribe("local average heading", 10, local_average_heading_publisher);

    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);
    posePublish = mNH.advertise<std_msgs::String>(("poses"), 10, true);
    global_average_heading_publisher = mNH.advertise<std_msgs::String>("global average heading", 10, true);
    local_average_heading_publisher = mNH.advertise<std_msgs::String>("local average heading", 10, true);

    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;
    std_msgs::String pose_msg;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
        case STATE_MACHINE_TRANSLATE:
        {
            state_machine_msg.data = "TRANSLATING";//, " + converter.str();
            float angular_velocity = 0.2;
            float linear_velocity = 0.1;
            setVelocity(linear_velocity, angular_velocity);
            break;
        }
        default:
        {
            state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
            break;
        }
        }

    }
    else
    { // mode is NOT auto

        // publish current state for the operator to see rotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
    stateMachinePublish.publish(state_machine_msg);

    std::stringstream rover_info;
    rover_info << rover_name << ", ";
    rover_info << current_location.x << ", ";
    rover_info << current_location.y << ", ";
    rover_info << current_location.theta;
    pose_msg.data = rover_info.str();
    posePublish.publish(pose_msg);
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
    // the rover's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();

    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
    // Only used if we want to take action after seeing an April Tag.
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( message->data > 0 )
    {
        if (message->data == 1)
        {
            // obstacle on right side
        }
        else
        {
            //obstacle in front or on left side
        }
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    current_location.x = message->pose.pose.position.x;
    current_location.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulation_mode == 0 || simulation_mode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!is_published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        is_published_name = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
             current_time);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void messageHandler(const std_msgs::String::ConstPtr& message)
{
}

void poseHandler(const std_msgs::String::ConstPtr &message)
{
    string pose_msg = message->data.c_str();
    string delimiter = ", ";
    std::string::size_type delimSize = pose_msg.find(delimiter);
    string name;
    double x, y, theta;
    bool nameParsed, xParsed, yParsed, thetaParsed = false;

    // message: "roverName, Xlocation, Ylocation, theta"
    // get data from message
    if (!nameParsed && !xParsed && !yParsed && !thetaParsed) {
        name = pose_msg.substr(0, delimSize);
        pose_message.erase(0, delimSize + delimiter.length());
        nameParsed = true;
    }
    if (nameParsed && !xParsed && !yParsed && !thetaParsed) {
        x = pose_msg.substr(0, delimSize);
        pose_message.erase(0, delimSize + delimiter.length());
        xParsed = true;
    }
    if (nameParsed && xParsed && !yParsed && !thetaParsed) {
        y = pose_msg.substr(0, delimSize);
        pose_message.erase(0, delimSize + delimiter.length());
        yParsed = true;
    }
    if (nameParsed && xParsed && yParsed && !thetaParsed) {
        theta = pose_msg.substr(0, delimSize);
//        pose_message.erase(0, delimSize + delimiter.length());
        thetaParsed = true;
    }

    // update pose or add new rover
    bool roverFound = false;
    if (nameParsed && xParsed && yParsed && thetaParsed) {
        for (int i = 0; i < roverList.size(); i++) {
            if(roverList[i].roverName.equals(name) && !roverFound) {
                roverList[i].setPose(x, y, theta);
                roverFound = true;
            }
        }
        if (!roverFound) {
            roverList.emplace_back(new Rover(name, x, y, theta));
        }
    }

    // update global average heading
    std_msgs::String gbl_avg_heading_msg;
    std::stringstream gbl_avg_heading_stream;

    gbl_avg_heading_stream << "Global average heading: " << calculateGlobalAverageHeading();
    gbl_avg_heading_msg.data = gbl_avg_heading_stream.str();

    global_average_heading_publisher.publish(gbl_avg_heading_msg);

    // update local average headings for each rover
    std_msgs::String local_avg_heading_msg;
    std::stringstream local_avg_heading_stream;

    for (int i = 0; i < roverList.size(); i++) {
        local_avg_heading_stream << "Local average heading for " << roverList[i].getName() << ": " << calculateLocalAverageHeading(roverList[i]);
        local_avg_heading_msg.data = local_avg_heading_stream.str();

        local_average_heading_publisher.publish(local_avg_heading_msg);
    }
}


void global_average_heading_handler(const std_msgs::String::ConstPtr &message) {

}

void local_average_heading_handler(const std_msgs::String::ConstPtr &message) {

}


















