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

#include "RotationalController.h"
#include "TranslationalController.h"
#include "SearchController.h"
#include "Pose.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h> 

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;

TranslationalController translational_controller;
RotationalController rotational_controller;
RotationalController rotational_translational_controller;
SearchController search_controller;

string rover_name;
char host[128];
bool published_name = false;

int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;
pose goal_location;


bool avoiding_obstacle = false;
vector <pose> saved_positions;
vector <pose> cluster_detect;

std_msgs::Int16 id_claimed_target;
std_msgs::Int16 id_claimed_target_once_detected; //ID of the claimed target once it has been detected by gatherer.

const int HOME_APRIL_TAG_ID = 256;
const int TOTAL_NUMBER_RESOURCES = 256;
bool targets_collected[TOTAL_NUMBER_RESOURCES];     // array of booleans indicating whether each target ID has been found and delivered
bool targets_detected[TOTAL_NUMBER_RESOURCES];      // array of booleans indicating whether each target ID has been found
bool targets_available[TOTAL_NUMBER_RESOURCES];     // array of booleans indicating whether each target ID has not been claimed by a collector.
bool targets_available_detected[TOTAL_NUMBER_RESOURCES]; // array of booleans indicating whether each target ID has been found and has not been claimed by a collector.
bool targets_home[TOTAL_NUMBER_RESOURCES]; // array of booleans indicating wheter each target ID has been delivered to home base.
bool targets_currently_claimed[TOTAL_NUMBER_RESOURCES]; // array of booleans indicating whether each target ID is currently claimed by a collector.
pose target_positions[TOTAL_NUMBER_RESOURCES];


// ajax and aeneas cluster detection
int targets_detected_screenshot = 0;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// sorted queue
vector <pose> queue;

#define LOGIC_INIT          0
#define LOGIC_SEARCH        1
#define LOGIC_COLLECT_RWALK 2
// state machine states
#define STATE_MACHINE_INITIALIZE 0
#define STATE_MACHINE_ROTATE    1
#define STATE_MACHINE_TRANSLATE 2
#define STATE_MACHINE_POP_WAYPOINT 3
#define STATE_MACHINE_SCORE_TARGET 4
#define STATE_MACHINE_CLAIM_TARGET 5
#define STATE_MACHINE_RETURN_HOME 6
#define STATE_MACHINE_EXPLORE_NEARBY 7
#define STATE_MACHINE_CHANGE_MODE 8
#define STATE_MACHINE_RANDOM_SEARCH 9
#define STATE_MACHINE_EXPLORE_NEAR_ORIGIN 10
int state_machine_state = STATE_MACHINE_INITIALIZE;

#define MODE_SEARCHER 0 // mode variable for rovers which will be used to perform lawnmower search.
#define MODE_COLLECTOR 1 // mode variable for rovers which will be used to perform random search and collect tags found by lawnmower searcher.
int rover_current_mode = MODE_SEARCHER; // initiall set all rovers to be collectors

#define CAPACITY_EMPTY 0 // indicates that rover has not claimed and is not carrying april tag
#define CAPACITY_CLAIMED 1 // indicates that rover has claimed an april tag, but hasn't found it yet.
#define CAPACITY_CARRYING 2 // inidcates that rover hs claimed an april tag, and is returning it to home base.
int roverCapacity = CAPACITY_EMPTY;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;
ros::Subscriber messageSubscriber;

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
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr &message);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);

//Utility functions
double computeGoalTheta(pose goal_location, pose current_location);
double computeDistanceBetweenWaypoints(pose final_location, pose start_location);
void setGoalLocation(pose new_goal_location);
int findIDClosestAvailableTarget();
void debugWaypoints();
void debugRotate();
void debugTranslate(double distance_);
void debugRandom();
void visitRandomLocation();
void claimResource(int resouceID);
void unclaimResource(int resouceID);
void homeResource(int resouceID);
bool isGoalReachedR(pose current_location, pose goal_location);
bool isGoalReachedT(pose current_location, pose goal_location);
std_msgs::Int16 parseMessage(vector<string> msg_parts);

bool cluster = false;
void circle();
void lookBack();

void debugTargetScreenshot(int count);

void reportDetected(std_msgs::Int16 msg);

void detectedMessage(vector<string> msg_parts);
void claimMessage(vector<string> msg_parts);
void unclaimMessage(vector<string> msg_parts);
void homeMessage(vector<string> msg_parts);

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    id_claimed_target.data = -1; // initialize target claimed
    id_claimed_target_once_detected.data = -1; // intially the gatherer has no claimed targets so -1 indicates that the gatherer is either looking for a claimed target or there is no current claimed target.

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    search_controller = SearchController(rover_name);
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);

    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((rover_name + "/targetPickUpImage"), 1, true);
    targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((rover_name + "/targetDropOffImage"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);

    for(int i = 0; i < TOTAL_NUMBER_RESOURCES; i++)
    {
        targets_available[i] = true;  // initially all targets are available to be claimed although none have been detected yet.
    }
    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the firt time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
        case STATE_MACHINE_INITIALIZE:
        {
            state_machine_msg.data = "INITIALIZING";
            if (rover_name == "ajax" || rover_name == "aeneas") // set mode to searcher
            {
                rover_current_mode = MODE_SEARCHER; // set ajax and aeneas to be the lawnmower path searchers.
                pose new_goal_location = search_controller.getNextWaypoint(current_location);
                setGoalLocation(new_goal_location);
            }
            else // all other robots, assign to collect
            {
                rover_current_mode = MODE_COLLECTOR;
                setGoalLocation(current_location);
            }
            state_machine_state = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_ROTATE:
        {
            float angular_velocity = 0.0;
            double linear_velocity = 0.0;
            if (isGoalReachedR(current_location, goal_location))
            {
                state_machine_state = STATE_MACHINE_TRANSLATE; // move to translate step
            }
            else
            {
                linear_velocity = rng->uniformReal(-.02, 0.02);
                angular_velocity = rotational_controller.calculateVelocity(current_location, goal_location);
            }
            setVelocity(linear_velocity, angular_velocity);

            std::stringstream converter;
            converter << 1;
            std_msgs::String message;
            message.data = "ANGULAR INFO: start " + converter.str();
            angular_publisher.publish(message);

            state_machine_msg.data = "ROTATING";
            break;
        }
        case STATE_MACHINE_TRANSLATE:
        {
            double angular_velocity = 0.0;
            double linear_velocity = 0.0;
            if (isGoalReachedT(current_location, goal_location))
            {
                // This is where the magic happens.  We must decide what to do once we have reached the desired location.
                if ( avoiding_obstacle )
                {
                    // this is part of assigning new goal, but it is taking into consideration, obstacles
                    avoiding_obstacle = false;
                    while (saved_positions.size() > 1)
                    {
                        saved_positions.pop_back();
                    }
                    goal_location.x = saved_positions.back().x;
                    goal_location.y = saved_positions.back().y;
                    goal_location.theta = computeGoalTheta(goal_location, current_location);
                    saved_positions.pop_back();
                    state_machine_state =STATE_MACHINE_ROTATE;
                }
                else if ( (rover_current_mode == MODE_SEARCHER) && (rover_name == "ajax") && !search_controller.isSearchFinished() )
                {
                    state_machine_state = STATE_MACHINE_POP_WAYPOINT; // We are ajax and we are currently searching with remaining waypoints
                }
                else if ( (rover_current_mode == MODE_SEARCHER) && (rover_name == "aeneas") && !search_controller.isSearchFinished() )
                {
                    state_machine_state = STATE_MACHINE_POP_WAYPOINT; // We are aeneas and we are currently searching with remaining waypoints
                }
                else if ( (rover_current_mode == MODE_SEARCHER) && (rover_name == "ajax") && search_controller.isSearchFinished() )
                {
                    state_machine_state = STATE_MACHINE_CHANGE_MODE; // We are ajax and we are currently searching with no remaining waypoints
                }
                else if ( (rover_current_mode == MODE_SEARCHER) && (rover_name == "aeneas") && search_controller.isSearchFinished() )
                {
                    state_machine_state = STATE_MACHINE_CHANGE_MODE; // We are aeneas and we are currently searching with no remaining waypoints
                }
                else if ( (rover_current_mode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_CLAIMED) )
                {
                    state_machine_state = STATE_MACHINE_EXPLORE_NEARBY; // We have gone to where we thought the target should be but didn't find it there.
                }
                else if ( (rover_current_mode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_EMPTY) )
                {
                    state_machine_state = STATE_MACHINE_CHANGE_MODE;
                }
                else if ( (rover_current_mode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_CARRYING) )
                {
                    state_machine_state = STATE_MACHINE_EXPLORE_NEAR_ORIGIN; // We have gone to where we thought the origin should be but didn't find it there.
                }
            }
            else
            {
                linear_velocity = translational_controller.calculateVelocity(current_location, goal_location);
                angular_velocity = rotational_translational_controller.calculateVelocity(current_location, goal_location);
            }
            setVelocity(linear_velocity, angular_velocity);
            state_machine_msg.data = "TRANSLATING";//, " + converter.str();
            break;
        }
        case STATE_MACHINE_POP_WAYPOINT:
        {
            state_machine_msg.data = "POPPING WAYPOINT";
            pose new_goal_location = search_controller.getNextWaypoint(current_location);
            setGoalLocation(new_goal_location);
            state_machine_state = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_CHANGE_MODE:
        {
            state_machine_msg.data = "CHANGING MODE";
            rover_current_mode = MODE_COLLECTOR; // change mode from lawnmower searcher to collector.
            state_machine_state = STATE_MACHINE_CLAIM_TARGET;
            break;
        }
        case STATE_MACHINE_EXPLORE_NEARBY:
        {
            state_machine_msg.data = "EXPORING NEARBY";
            if (targets_home[id_claimed_target.data])
            {
                // This means that someone else has taken this target home alread, so something went wrong.  Drop it and claim a new target.
                state_machine_state = STATE_MACHINE_CLAIM_TARGET;
            }
            else
            {
                // It must be around here somewhere.  Keep searching for it.
                double r = rng->uniformReal(0.0, 1.0);
                double t = rng->uniformReal(0, 2 * M_PI);
                goal_location.x = target_positions[id_claimed_target.data].x + r * cos(t); // Explore in a 1m circle around where you think the target is located.
                goal_location.y = target_positions[id_claimed_target.data].y + r * sin(t);
                goal_location.theta = computeGoalTheta(goal_location, current_location);
                state_machine_state = STATE_MACHINE_ROTATE;
            }
            break;
        }
        case STATE_MACHINE_RETURN_HOME:
        {
            state_machine_msg.data = "RETURNING TO HOME";
            goal_location.x = 0.0;
            goal_location.y = 0.0;
            goal_location.theta = computeGoalTheta(goal_location, current_location);
            state_machine_state = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_CLAIM_TARGET:
        {
            state_machine_msg.data = "CLAIMING TARGET";
            int result = findIDClosestAvailableTarget(); // search targets detected and avialble
            if(result != -1)
            {
                claimResource(result); // update targets detected and available array
                id_claimed_target.data = result; // This should be where targetClaimed gets set.
                roverCapacity=CAPACITY_CLAIMED; // Update status of rover
                setGoalLocation(target_positions[result]);
                state_machine_state = STATE_MACHINE_ROTATE;
            }
            else
            {
                state_machine_state = STATE_MACHINE_RANDOM_SEARCH;
            }
            break;
        }
        case STATE_MACHINE_RANDOM_SEARCH:
        {
            state_machine_msg.data = "RANDOMLY SEARCHING";
            visitRandomLocation();
            state_machine_state = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_EXPLORE_NEAR_ORIGIN:
        {
            state_machine_msg.data = "EXPORING NEAR ORIGIN";
            double r = rng->uniformReal(0.0, 1);
            double t = rng->uniformReal(0, 2 * M_PI);
            goal_location.x = 0.0 + r * cos(t); // Explore in a 1m circle around where you think the target is located.
            goal_location.y = 0.0 + r * sin(t);
            goal_location.theta = computeGoalTheta(goal_location, current_location);
            state_machine_state = STATE_MACHINE_ROTATE;
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

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill swith timer reaches kill_switch_timeout seconds
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
    for (int message_index = 0; message_index < message->tags.data.size(); message_index++) {
        int tag_id = message->tags.data[message_index];
        if (tag_id == HOME_APRIL_TAG_ID && roverCapacity==CAPACITY_CARRYING) {
            setVelocity(0.0,0.0); // stop the rover
            homeResource(id_claimed_target.data);
            targetDropOffPublish.publish(message->image);
            roverCapacity = CAPACITY_EMPTY;
            id_claimed_target.data = -1;
            state_machine_state = STATE_MACHINE_CLAIM_TARGET;
        }
        else if (tag_id != HOME_APRIL_TAG_ID && !targets_home[tag_id]) {
            if (rover_current_mode == MODE_COLLECTOR) {
                switch(roverCapacity){
                case CAPACITY_CLAIMED:
                    if (tag_id != id_claimed_target.data) {
                        unclaimResource(id_claimed_target.data);
                        claimResource(tag_id); // update targets detected and available array
                        id_claimed_target.data = tag_id; // This should be where targetClaimed gets set.
                    }
                    break;
                case CAPACITY_EMPTY:
                    claimResource(tag_id); // update targets detected and available array
                    id_claimed_target.data = tag_id; // This should be where targetClaimed gets set.
                    break;
                }
                if (roverCapacity!=CAPACITY_CARRYING)
                {
                    setVelocity(0.0,0.0); // stop the rover
                    roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying
                    id_claimed_target_once_detected.data = tag_id;
                    // publish to scoring code
                    targetPickUpPublish.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    targetCollectedPublish.publish(id_claimed_target_once_detected);
                    // targetCollectedPublish.publish(targetDetected); // from UNM Code
                    state_machine_state = STATE_MACHINE_RETURN_HOME;
                }
            }
            if (!targets_detected[tag_id]) {
                std_msgs::Int16 tag;
                tag.data = tag_id;
                reportDetected(tag);
            }
        }

    }
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
        setVelocity(-0.2,0.0); // First back up a little to give space before taking action.
        if(rover_current_mode==MODE_COLLECTOR ) //!(avoiding_obstacle)
        {
            pose savedPosition;
            savedPosition.x = goal_location.x;
            savedPosition.y = goal_location.y;
            savedPosition.theta = goal_location.theta;
            saved_positions.push_back(savedPosition);
            if ( (roverCapacity==CAPACITY_CARRYING) && (angles::shortest_angular_distance(current_location.theta, atan2(-current_location.y, -current_location.x)) < M_PI_2) )
            {
                // We are going to the goal with a target.  Just wait till the other guy moves out of our way.
                // Try not to move.
                goal_location.x = current_location.x;
                goal_location.y = current_location.y;
                goal_location.theta = current_location.theta;
            }
            else
            {
                // We are not delivering a target so lets get out of the way.
                // obstacle on right side
                if (message->data == 1)
                {
                    goal_location.theta = current_location.theta + 0.78;
                }
                //obstacle in front or on left side
                else
                {
                    goal_location.theta = current_location.theta - 0.78;
                }
                //select new position 0.75 m from current location
                goal_location.x = current_location.x + (0.75 * cos(goal_location.theta));
                goal_location.y = current_location.y + (0.75 * sin(goal_location.theta));
            }
            avoiding_obstacle = true;
            //switch to reset rotate pid to trigger collision avoidance
            state_machine_state = STATE_MACHINE_ROTATE;
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
    if (!published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        published_name = true;
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

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr &message)
{
    targets_collected[message->data] = 1;
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

double computeGoalTheta(pose goalLocation, pose currentLocation)
{
    return atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
}

double computeDistanceBetweenWaypoints(pose final_location, pose start_location)
{
    return fabs(sqrt(pow(final_location.x - start_location.x, 2.0) + pow((final_location.y - start_location.y), 2.0)));
}

void setGoalLocation(pose new_goal_location)
{
    goal_location.x = new_goal_location.x;
    goal_location.y = new_goal_location.y;
    goal_location.theta = computeGoalTheta(goal_location, current_location);
}

void reportDetected(std_msgs::Int16 msg) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    int targets_detected_size = 0;
    for(int i = 0; i < TOTAL_NUMBER_RESOURCES; i++)
    {
        if(targets_detected[i])
        {
            targets_detected_size++;
        }
    }
    std::stringstream converter;
    
    converter <<
                 rover_name << " " <<
                 msg.data << " " <<
                 current_location.x << " " <<
                 current_location.y << " " <<
                 current_location.theta << " " <<
                 current_time-time_stamp_transition_to_auto << " " <<
                 targets_detected_size;
    
    std_msgs::String message;
    message.data = "D " + converter.str();
    messagePublish.publish(message);
}

void claimResource(int resouceID) // simply publish (broadcast) 
{
    double current_time = ros::Time::now().toSec();
    std::stringstream converter;
    converter << resouceID << " " << rover_name << " " << current_time-time_stamp_transition_to_auto;
    std_msgs::String message;
    message.data = "C " + converter.str(); // claim this token for pickup
    messagePublish.publish(message);
}

void unclaimResource(int resouceID) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    std::stringstream converter;
    converter << resouceID << " " << rover_name << " " << current_time-time_stamp_transition_to_auto;
    std_msgs::String message;
    message.data = "U " + converter.str(); // unclaim this token for pickup
    messagePublish.publish(message);
}

void homeResource(int resouceID) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    int targets_home_size = 0;

    for(int i = 0; i < TOTAL_NUMBER_RESOURCES; i++)
    {
        if(targets_home[i])
        {
            targets_home_size++;
        }
    }

    std::stringstream converter;
    converter << resouceID << " " << rover_name << " " << current_time-time_stamp_transition_to_auto << " " << targets_home_size;
    std_msgs::String message;
    message.data = "H " + converter.str(); // unclaim this token for pickup
    messagePublish.publish(message);
}

// search detected targets and find the shortest distance to your current position
int findIDClosestAvailableTarget()
{
    double min_distance = LONG_MAX;
    int id_closest_target = -1;
    vector<float> claimed_directions;
    for(int ii = 0; ii < TOTAL_NUMBER_RESOURCES; ii++)
    {
        if(targets_currently_claimed[ii])
        {
            claimed_directions.push_back(atan2(target_positions[ii].y, target_positions[ii].x));
        }
    }
    for(int resource = 0; resource < TOTAL_NUMBER_RESOURCES; resource++)
    {
        if(targets_available_detected[resource])
        {
            if (claimed_directions.size() > 0)
            {
                bool flag = true;
                for ( int itr = 0; itr < claimed_directions.size(); itr++ )
                {
                    if(fabs(claimed_directions[itr] - atan2(target_positions[resource].y - current_location.y, target_positions[resource].x - current_location.x)) < 0.25)
                    {
                        flag = false;
                    }
                }
                if(flag)
                {
                    double current_distance = computeDistanceBetweenWaypoints(target_positions[resource], current_location);
                    if(current_distance < min_distance)
                    {
                        min_distance = current_distance;
                        id_closest_target = resource;
                    }
                }
            }
            else
            {
                double current_distance = computeDistanceBetweenWaypoints(target_positions[resource], current_location);
                if(current_distance < min_distance)
                {
                    min_distance = current_distance;
                    id_closest_target = resource;
                }
            }
        }
    }
    return id_closest_target;
}

/***********************
 * CUSTOM MESSAGE HANDLERS
 ************************/
void messageHandler(const std_msgs::String::ConstPtr& message)
{
    string msg = message->data;

    size_t type_pos = msg.find_first_of(" ");
    string type = msg.substr(0, type_pos);
    msg = msg.substr(type_pos+1);

    vector<string> msg_parts;

    size_t cur_tok = msg.find_first_of(" ");;
    while(cur_tok != string::npos) { // until end of string
        msg_parts.push_back(msg.substr(0, cur_tok));
        msg = msg.substr(cur_tok + 1);
        cur_tok = msg.find_first_of(" ");
    }

    msg_parts.push_back(msg);

    if(type == "D")
    {
        detectedMessage(msg_parts);
    }
    else if(type == "C")
    {
        claimMessage(msg_parts);
    }
    else if(type == "U")
    {
        unclaimMessage(msg_parts);
    }
    else if(type == "H")
    {
        homeMessage(msg_parts);
    }
}

void detectedMessage(vector<string> msg_parts)
{
    std_msgs::Int16 tmp;
    tmp.data = -1;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    string name = "";

    stringstream converter;

    converter << msg_parts[0];
    converter >> name;
    converter.str("");
    converter.clear();

    converter << msg_parts[1];
    converter >> tmp.data;
    converter.str("");
    converter.clear();

    converter << msg_parts[2];
    converter >> x;
    converter.str("");
    converter.clear();

    converter << msg_parts[3];
    converter >> y;
    converter.str("");
    converter.clear();

    converter << msg_parts[4];
    converter >> theta;
    converter.str("");
    converter.clear();

    // lookup tables
    target_positions[tmp.data].x = x + 0.75 * cos(theta); // project forward by 0.5 meters in the direction that the finding rover was facing when the target was detected
    target_positions[tmp.data].y = y + 0.75 * sin(theta);
    target_positions[tmp.data].theta = theta;
    targets_detected[tmp.data] = true;
    targets_available_detected[tmp.data] = targets_detected[tmp.data] && targets_available[tmp.data];
    if ( (rover_current_mode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_EMPTY) )
    {
        state_machine_state = STATE_MACHINE_CHANGE_MODE;
    }
}

void claimMessage(vector<string> msg_parts) // claimed this, now remove from global queue
{
    std_msgs::Int16 tag_id = parseMessage(msg_parts);

    targets_currently_claimed[tag_id.data] = true;
    targets_available[tag_id.data] = false;
    targets_available_detected[tag_id.data] = targets_detected[tag_id.data] && targets_available[tag_id.data];
}

void unclaimMessage(vector<string> msg_parts) // unclaimed this, now add back to global queue
{
    std_msgs::Int16 tag_id = parseMessage(msg_parts);
    targets_currently_claimed[tag_id.data] = false;
    targets_available[tag_id.data] = true;
    targets_available_detected[tag_id.data] = targets_detected[tag_id.data] && targets_available[tag_id.data];
}

void homeMessage(vector<string> msg_parts) // unclaimed this, now add back to global queue
{
    std_msgs::Int16 tag_id = parseMessage(msg_parts);
    targets_home[tag_id.data] = true;
    targets_currently_claimed[tag_id.data] = false;
}

std_msgs::Int16 parseMessage(vector<string> msg_parts) {
    std_msgs::Int16 tmp;
    tmp.data = -1;

    stringstream converter;

    converter << msg_parts[0];
    converter >> tmp.data;
    converter.str("");
    converter.clear();
    return tmp;
}

void visitRandomLocation()
{
    double r = rng->uniformReal(0.5, 6);
    double t = rng->uniformReal(0, 2 * M_PI);
    goal_location.x = r * cos(t);
    goal_location.y = r * sin(t);
    goal_location.theta = computeGoalTheta(goal_location, current_location);
}

bool isGoalReachedR(pose current_location, pose goal_location)
{
    float distance_to_goal = fabs(angles::shortest_angular_distance(current_location.theta, goal_location.theta));
    return distance_to_goal < 0.25; // 0.25 radians around goal heading
}

bool isGoalReachedT(pose current_location, pose goal_location)
{
    float distance_to_goal = hypotf(goal_location.x-current_location.x, goal_location.y-current_location.y);
    return distance_to_goal < 0.5; // 0.5 meter circle around target waypoint.
}
