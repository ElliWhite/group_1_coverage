//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * quadLRTA.cpp
 *
 *  Created on: May 6, 2014
 *      Author: francescow
 */

#include <cstring>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>

#include <boost/thread/mutex.hpp>

#include <actionlib/client/simple_action_client.h>
#include <path_following_msgs/WaypointFollowAction.h>

#include "quadcopterRosCtrl.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadcopter_ctrl/LRTAmsg.h"
#include "quadcopter_ctrl/OSmsg.h"
#include "quadcopter_ctrl/HelpRequest.h"
#include "std_msgs/Int64.h"

#include "termColors.h"
#include "PathPlanningAlgNew.h"
#include "LRTAstar.h"
#include "Log.h" 
#include "BoostGraphWrapper.h"


#define LogRobot LogInfo << "robot [" << my_robot_id << ", " << RobotTypeStr[my_robot_type] << "]: "  
#define LogRobotDebug LogDebug << "robot [" << my_robot_id << ", " << RobotTypeStr[my_robot_type] << "]: "  

using std::cout;
using std::endl;
using std::vector;


quadcopter_ctrl::LRTAmsg LRTAinfo;
quadcopter_ctrl::OSmsg osInfo;
geometry_msgs::PoseStamped quadPos;
geometry_msgs::PoseStamped targetPos;
geometry_msgs::PoseStamped targetPosNext;
geometry_msgs::PoseStamped subTarget;
std::string global_frame_id;
std_msgs::Int64 controlSignal;
bool help_request_received = false;

int my_robot_id = 0;
int my_robot_type = 0;

const std::string RobotTypeStr[] = {"uav", "ugv", "none"};

enum RobotType
{
    kRobotTypeUav = 0, kRobotTypeUgv, kRobotTypeNone, kRobotTypeNums
};

class RobotState
{
public:

    RobotState() : bEngaged(false)
    {
        planState.robot_id = -1; // invalid 
    }

    void UpdateCurrNode()
    {
        planState.currNode = planState.nextNode;
    }

    quadcopter_ctrl::LRTAmsg planState;
    bool bEngaged;
};

boost::mutex vec_robot_state_mutex; // vec_robot_state is not guaranteed to be thread-safe!
std::vector<RobotState> vec_robot_state;
std::set<int> setEngagedNodes;

LRTAstar myLRTA;

// update current node in both robot states vector and LRTA* manager

void UpdateCurrNode()
{
    myLRTA.updateCurrNode();

    // lock the mutex for accessing vec_robot_state
    boost::unique_lock<boost::mutex> scoped_lock(vec_robot_state_mutex);
    if (vec_robot_state.size() > my_robot_id)
        vec_robot_state[my_robot_id].UpdateCurrNode();
}

double zHeight = 0;
double MAP_SCALE, OFS_X, OFS_Y, WP_STEP, CRIT_DIST, threshold;
int quadPosAcquired = 0;
bool loaded = 0;
std::vector<int> path;

///FUNCTIONS
std::string get_selfpath(void);
std::string add_argv(std::string str, char* argvalue);
void publishTarget(ros::Publisher& countPub);
void publishSubTarget(ros::Publisher& posPub);

RobotType getRobotType(const std::string& type_str);
template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue);

/// < callbacks
void quadPosFromVrep(const geometry_msgs::PoseStamped::ConstPtr& pubQuadPose);
void updateCount(const quadcopter_ctrl::LRTAmsg::ConstPtr& LRTAinfo);

void helpRequestCallback(const quadcopter_ctrl::HelpRequest::ConstPtr& request)
{
    LogInfo << "Callback request for help" << endl;
    if(my_robot_id != request->robot_id) 
    {
        return; 
    }
    while (!help_request_received)
    {
        controlSignal.data = request->node;
        help_request_received = true;
        loaded = 0;
        cout << "help_request_received status " << help_request_received << endl;

        ros::Duration(0.01).sleep();
    }
}

/// < work procedures 
bool ugvWork(bool& loaded, ros::Publisher& updateCount_pub,
             ros::Publisher& targetObjPos_pub,
             actionlib::SimpleActionClient<path_following_msgs::WaypointFollowAction>& ac);
bool uavWork(bool& loaded, std::string& controlMode, int& inSubPath,
             ros::Publisher& updateCount_pub, ros::Publisher& targetObjPos_pub);
bool uavWorkHelpRequest(bool& loaded, int& inSubPath, int& wpIndex, BoostGraphWrapper& graphWrapper,
                        ros::Publisher& updateCount_pub, ros::Publisher& targetObjPos_pub);

bool computeEngagedNodes();
void print(std::set<int>& setin);

int main(int argc, char **argv)
{
    /// < argv[1] contains the ID number of the robot to be controlled (0,1,2...)
    if (argc < 7)
    {
        printf("%s** ERROR **\n"
               "argv[1]: Quadcopter # to control\n"
               "argv[2]: Input file\n"
               "argv[3]: zHeight of flight\n"
               "argv[4]: Control Mode ('sim' or 'asctec')\n"
               "argv[5]: STARTNODE, index of first node\n"
               "argv[6]: Min #visits for each node (1=\"simple coverage\")%s\n", TC_RED, TC_NONE);
        exit(EXIT_FAILURE);
    }

    my_robot_id = static_cast<int> (strtol(argv[1], NULL, 10)); // get the robot id
    printf("%s[%s] My robot id is: %d%s\n", TC_YELLOW, argv[1], my_robot_id, TC_NONE);

    /// In this way each robot flies at a different height
    zHeight = static_cast<double> (strtod(argv[3], NULL));
    printf("%s[%s] My zHeight is: %f%s\n", TC_YELLOW, argv[1], zHeight, TC_NONE);

    int startNode = strtol(argv[5], NULL, 0);

    std::string filename(argv[2]);
    std::string folder_path = get_selfpath();
    //std::string file_path = folder_path + "/Input/Grids/" + filename;
    std::string file_path = folder_path + "/../../../src/quadcopter_ctrl/Input/Grids/" + filename;
    //cout << "file path: " << file_path << endl; 

    /// < load access matrix from file 
    std::ifstream access_matrix;
    access_matrix.open(file_path.c_str());
    if (!access_matrix.is_open())
    {
        printf("%sAccess matrix not found! (sure is the executable folder?)%s\n", TC_RED, TC_NONE);
        exit(EXIT_FAILURE);
    }
    /*
      std::string posV_filename = "posV_" + filename;
      std::string posV_file_path = folder_path + "/Input/PosV/" + posV_filename;
      std::ifstream pos_Vec;
      pos_Vec.open( posV_file_path.c_str() );
      if( !pos_Vec.is_open() ){
        printf("%sPos_Vec matrix not found!%s\n", TC_RED, TC_NONE);
        exit(EXIT_FAILURE);
      }
     */
    int min_visit = strtol(argv[6], NULL, 0);
    cout << "min_visit: " << min_visit << endl;

    /// < inialize the graph 
    myLRTA.init_acc(access_matrix, startNode, min_visit);
    //myLRTA.init_graph_pos(access_matrix, pos_Vec, startNode, min_visit);    //Constructor inputs is (graph, position of nodes in space)

    std::string controlMode(argv[4]);
    printf("%s[%s] Control Mode: %s%s\n", TC_YELLOW, argv[3], controlMode.c_str(), TC_NONE);
    if (PathPlanningAlgNew::LoadParams(controlMode, MAP_SCALE, OFS_X, OFS_Y, WP_STEP, CRIT_DIST, threshold))
    {
        printf("%s** INCORRECT CONTROL MODE **%s\n", TC_RED, TC_NONE);
        exit(EXIT_FAILURE);
    }

    double dist;
    int running = 1;
    int inSubPath = 0;
    //bool loaded = 0;
    controlSignal.data = -1;
    int wpIndex = 0;

    /// ROS NETWORK CONFIGURATION ///
    /* The following strings are used to concatenate the topic name to the argument passed to
     * the node (the argv[1]), so to name each node with a different name and send signals to
     *  different topics.
     * (e.g. if argv[1] = 0 the node will be named quadcopterRosCtrl_0, publish to
     *  vrep/targetObjPos_0, etc...)
     */
    //std::string nodeName = add_argv("quadLRTAstar", argv[1]);
    //std::string targetObjPosName = add_argv("targetObjPos", argv[1]);
    //std::string quadcopPosName = add_argv("quadcopPos", argv[1]);
    std::string targetObjPosName;
    std::string quadcopPosName;
    std::string server_name;


    //ros::init(argc, argv, nodeName);
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle n;

    /// < private NodeHandle 
    ros::NodeHandle n_("~");
    quadcopPosName = getParam<std::string>(n_, "robot_pose", "/pose");
    targetObjPosName = getParam<std::string>(n_, "target_robot_pose", "/target_pose");
    global_frame_id = getParam<std::string>(n_, "global_frame_id", "/world");
    server_name = getParam<std::string>(n_, "control_server_name", "path_following_");
    std::string robot_type_str = getParam<std::string>(n_, "robot_name", "none");
    std::string help_request_name = getParam<std::string>(n_, "help_request_name", "/help_request");

    my_robot_type = getRobotType(robot_type_str.substr(0, 3));
    printf("%s[%s] my robot type: %s%s\n", TC_YELLOW, argv[3], RobotTypeStr[my_robot_type].c_str(), TC_NONE);


    /// < Publishers & Subscribers 
    ros::Publisher targetObjPos_pub = n.advertise<geometry_msgs::PoseStamped>(targetObjPosName, 100); // publish my current target/subtarget 

    ros::Subscriber quadcopPos_sub = n.subscribe(quadcopPosName, 100, quadPosFromVrep); // get my pose from vrep

    ros::Publisher updateCount_pub = n.advertise<quadcopter_ctrl::LRTAmsg>("updateLRTACount", 100); // publish my planned target

    ros::Subscriber updateCount_sub = n.subscribe("updateLRTACount", 100, updateCount); // get planned targets from my team 

    ros::Publisher completed_pub = n.advertise<quadcopter_ctrl::OSmsg>("completedPath", 100); // publish I've completed my work 

    ros::Subscriber help_request_sub = n.subscribe(help_request_name, 100, helpRequestCallback);


    /// < Boost manager for the graph 

    BoostGraphWrapper graphWrapper(file_path);

    /// < Define loop rate 

    ros::Rate loop_rate(DEF_LOOP_RATE); //Loop at DEF_LOOP_RATE

    /*            *** PoseStamped structure: ***
    //            std_msgs/Header header
    //              uint32 seq
    //              time stamp
    //              string frame_id
    //            geometry_msgs/Pose pose
    //              geometry_msgs/Point position
    //                    float64 x
    //                    float64 y
    //                    float64 z
    //              geometry_msgs/Quaternion orientation
    //                    float64 x
    //                    float64 y
    //                    float64 z
    //                    float64 w
     */

    /// < Action clinet for ugvs
    //actionlib::SimpleActionClient<path_following_msgs::WaypointFollowAction> ac("path_following_ugv2",true);
    actionlib::SimpleActionClient<path_following_msgs::WaypointFollowAction> ac(server_name, true);


    while (ros::ok())
    {
        if (myLRTA.isCompleted() == false)
        {
            if (quadPosAcquired) // ok I received a valid position and the communication with position-feedback started 
            {
                quadPosAcquired = 0;
                //cout << "."; cout.flush();
                if (running == 0)
                {
                    printf("%s[%s] On my way Sir Captain!%s\n", TC_YELLOW, argv[1], TC_NONE);
                    running = 1;
                }

                switch (my_robot_type)
                {
                case kRobotTypeUgv:
                    ugvWork(loaded, updateCount_pub, targetObjPos_pub, ac);
                    break;

                case kRobotTypeUav:
                    
                    if (controlSignal.data != -1 && help_request_received) 
                    {
                        uavWorkHelpRequest(loaded,inSubPath, wpIndex, graphWrapper, updateCount_pub, targetObjPos_pub);
                    }
                    else
                    {
                        uavWork(loaded, controlMode, inSubPath, updateCount_pub, targetObjPos_pub);
                    }
                    break;

                default:
                    cout << "ERROR: you gave me a wrong type: " << my_robot_type << endl;
                    cout << "           your robot type str: " << robot_type_str << endl;
                    quick_exit(-1);
                }

            }
            else // still waiting to receive a valid position 
            {

                if (running == 1)
                {
                    printf("%s[%s] ** No incoming vrep/quadcopPos! (waiting...) **%s\n", TC_YELLOW, argv[1], TC_NONE);
                }
                running = 0;
            }

        }
        else // I completed the area coverage
        {
            printf("%s[%s] ** Area coverage completed! **%s\n", TC_GREEN, argv[1], TC_NONE);

            osInfo.ID = strtol(argv[1], NULL, 0);
            osInfo.numNodes = myLRTA.getNumFreeNodes();
            std::vector<int> finalPath = myLRTA.getFinalPath();
            // FIXME there is an issue with the last execution of the LRTA class:
            // the result is that an additional waypoint is added to the path so
            // we have to remove it before sending the information to the listener
            //finalPath.pop_back();
            osInfo.path = finalPath;
            //filename.resize(filename.size()-2); /// XXX REMEBER TO DELETE THIS LINE FIXME
            osInfo.fileName = "LRTA_" + filename;
            completed_pub.publish(osInfo);
            
            myLRTA.printCoverage();

            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



/// < work procedure for ugv

bool ugvWork(bool& loaded, ros::Publisher& updateCount_pub,
             ros::Publisher& targetObjPos_pub,
             actionlib::SimpleActionClient<path_following_msgs::WaypointFollowAction>& ac)
{
    if (loaded == 0)
    {
        LogRobot << "I'm reaching node " << myLRTA.getCurrentIndex() << endl; // NOTE: here current and next nodes coincide 

        loaded = 1;
        publishTarget(updateCount_pub);
        ac.waitForServer();

        path_following_msgs::WaypointFollowGoal waypoint;
        waypoint.target_pose = targetPos;
        targetObjPos_pub.publish(targetPos);
        ac.sendGoal(waypoint);
    }

    float distToTarget = fabs((PathPlanningAlgNew::Distance(&quadPos, &targetPosNext)));
    bool bArrived = distToTarget < 0.1;

    actionlib::SimpleClientGoalState state = ac.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        LogRobot << "Got in node " << myLRTA.getNextIndex() << /*", bArrived: " << bArrived <<*/ " dist: " << distToTarget;

        UpdateCurrNode();

        bool bStop = computeEngagedNodes();
        /*if(bStop) // this can be used to stop the robot instead of making it plan and move
        {
            return false; 
        }*/

        bool bGoAhead = myLRTA.findNext(setEngagedNodes);
        if (bGoAhead)
        {
            publishTarget(updateCount_pub);

            LogRobot << "I'm in node " << myLRTA.getCurrentIndex() << ", moving to " << myLRTA.getNextIndex() << endl;
            
            std::set<int> setNeighbours;
            myLRTA.getNeighboursSet(myLRTA.getCurrentIndex(), setNeighbours);
            

            ac.waitForServer();

            path_following_msgs::WaypointFollowGoal waypoint;
            //waypoint.target_pose = targetPos;
            waypoint.target_pose = targetPosNext;
            //targetObjPos_pub.publish(targetPos);
            targetObjPos_pub.publish(targetPosNext);
            ac.sendGoal(waypoint);
        }
    }
    return true;
}

/// < work procedures for uav 

bool uavWork(bool& loaded, std::string& controlMode, int& inSubPath,
             ros::Publisher& updateCount_pub, ros::Publisher& targetObjPos_pub)
{
        
    if (loaded == 0)
    {
        loaded = 1;

        LogRobot << "I'm reaching node " << myLRTA.getCurrentIndex()  << endl; // NOTE: here current and next nodes coincide 

        //cout << "current pos: " << quadPos.pose.position.x << ", " << quadPos.pose.position.y << ", " << quadPos.pose.position.z << endl;

        publishTarget(updateCount_pub);
        targetObjPos_pub.publish(targetPos);
        if (controlMode == "asctec") targetObjPos_pub.publish(targetPos);
    }

    // Calculating current l^2-norm between target and quadcopter (Euclidean distance)
    double dist = fabs(PathPlanningAlgNew::Distance(&quadPos, &targetPosNext));
    //cout << "Distance to target = " << dist << " m" << endl;

    /// < asctec mode
    if (controlMode == "asctec")
    {
        if (dist < threshold)
        {
            //printf("\n%s[%s] TARGET REACHED!%s\n", TC_GREEN, argv[1], TC_NONE);
            //sleep(5);
            myLRTA.findNext();
            publishTarget(updateCount_pub);
            targetObjPos_pub.publish(targetPos);
            //printf("\n%s[%s] NEXT TARGET SENT!%s\n", TC_GREEN, argv[1], TC_NONE);
        }
    }
    else /// < sim mode
    {
        if (inSubPath == 0)
        {
            if (dist > CRIT_DIST) // we are distant from target, need to create safe subtargets 
            {
                inSubPath = 1;
                publishSubTarget(targetObjPos_pub);
                //std::cout << "First subTarget Published!" << std::endl;
            }
            else if (dist < threshold) // we are close to target 
            {

                LogRobot << "Got in node " << myLRTA.getNextIndex() << /*", bArrived: " << bArrived <<*/ " dist: " << dist;

                UpdateCurrNode();
        
                //LogRobot << "Finding next node:" << endl;
                myLRTA.findNext();
                publishTarget(updateCount_pub);
                targetObjPos_pub.publish(targetPosNext);

                LogRobot << "I'm in node " << myLRTA.getCurrentIndex() << ", moving to " << myLRTA.getNextIndex() << endl;

                //In the following if, "dist" is calculated again since publishTarget changed targetPos
                //if (fabs(PathPlanningAlgNew::Distance(&quadPos, &targetPos)) < CRIT_DIST)
                //{
                //    targetObjPos_pub.publish(targetPos);
                //    //std::cout << "Target #" << wpIndex << " reached!" << std::endl;
                //}
            }
            else
            {
                //publishTarget(updateCount_pub);
                inSubPath = 1;
            }
        }
        else
        {
            double sub_dist = fabs((PathPlanningAlgNew::Distance(&quadPos, &subTarget)));
            if (dist < threshold)
            {
                inSubPath = 0;
                targetObjPos_pub.publish(targetPosNext);
            }
            else if (sub_dist < threshold)
            {
                publishSubTarget(targetObjPos_pub);
                //std::cout << "subTarget Published!" << std::endl;
            }
        }
    }
    return true;
}

bool uavWorkHelpRequest(bool& loaded, int& inSubPath, int& wpIndex, BoostGraphWrapper& graphWrapper,
                        ros::Publisher& updateCount_pub, ros::Publisher& targetObjPos_pub)
{

    if (loaded == 0)
    {
        LogRobot << "help requested, managing it"; 
        
        path.clear();
        graphWrapper.computeShortestPath(myLRTA.getCurrentIndex(), controlSignal.data, path);
        graphWrapper.printPath(path);
        loaded = 1;
        inSubPath = 0;
        wpIndex = 0;
        //int i = path[wpIndex] / wrapper.gridSizeY;
        //int j = path[wpIndex] % wrapper.gridSizeY;
        //targetPos.pose.position.x = static_cast<float>(i) * MAP_SCALE - OFS_X;
        //targetPos.pose.position.y = static_cast<float>(j) * MAP_SCALE - OFS_Y;
        //targetPos.pose.position.z = zHeight;
        //int i = myLRTA.getCurrentIndex() / graphWrapper.gridSizeY;
        //int j = myLRTA.getCurrentIndex() % graphWrapper.gridSizeY;
        
        // keep on going on the current target 
        //targetPos.pose.position.x = static_cast<float> (i) * MAP_SCALE - OFS_X;
        //targetPos.pose.position.y = static_cast<float> (j) * MAP_SCALE - OFS_Y;
        //targetPos.pose.position.z = zHeight;
        myLRTA.nextNode = path[wpIndex];
        
        publishTarget(updateCount_pub);
        targetObjPos_pub.publish(targetPosNext);
    }

    // Calculating current l^2-norm between target and quadcopter (Euclidean distance)
    float dist = fabs(PathPlanningAlgNew::Distance(&quadPos, &targetPosNext));
    //cout << "Distance to target = " << dist << " m" << endl;

    if (inSubPath == 0)
    {
        if (dist > CRIT_DIST)
        {
            inSubPath = 1;
            publishSubTarget(targetObjPos_pub);
            //std::cout << "First subTarget Published!" << std::endl;
        }
        else if (dist < threshold) // once you get in the current target start with new path 
        {
            ++wpIndex;
            if (wpIndex == (int) path.size())
            {
                help_request_received = false;
                controlSignal.data = -1;
                cout << "I have satisfied your request of help. Now I'll continue to cover the area" << endl;
            }
            else
            {
                std::cout << "Finding next for supporting the request of help:" << endl;
                //int i = path[wpIndex] / graphWrapper.gridSizeY;
                //int j = path[wpIndex -] % graphWrapper.gridSizeY;
                //targetPos.pose.position.x = static_cast<float> (i) * MAP_SCALE - OFS_X;
                //targetPos.pose.position.y = static_cast<float> (j) * MAP_SCALE - OFS_Y;
                //targetPos.pose.position.z = zHeight;
                myLRTA.currentNode = path[wpIndex - 1];
                myLRTA.nextNode = path[wpIndex];
                //LRTAinfo.currNode = myLRTA.currentNode;
                //LRTAinfo.nextNode = myLRTA.nextNode;
                //if (myLRTA.graphNodes.at(path[wpIndex]).nodeCount == 0)
                //{
                //    LRTAinfo.isNextVisited = 0;
                //}
                //else
                //{
                //    LRTAinfo.isNextVisited = 1;
                //}
                //cout << "target position: " << targetPos.pose.position.x << ", " << targetPos.pose.position.y << ", " << targetPos.pose.position.z << endl;
                //cout << "LRTAinfo.currNode: " << LRTAinfo.currNode << endl;
                //cout << "LRTAinfo.nextNode: " << LRTAinfo.nextNode << endl;
                //updateCount_pub.publish(LRTAinfo);
                
                publishTarget(updateCount_pub);
                targetObjPos_pub.publish(targetPosNext);

                LogRobot << "I'm in node " << myLRTA.getCurrentIndex() << ", moving to " << myLRTA.getNextIndex() << endl;
                

                //if (fabs(PathPlanningAlgNew::Distance(&quadPos, &targetPos)) < CRIT_DIST)
                //{
                //    targetObjPos_pub.publish(targetPos);
                //    //std::cout << "Target #" << wpIndex << " reached!" << std::endl;
                //}
            }
        }
        else
        {
            //updateTarget(updateCount_pub);
            inSubPath = 1;
        }
    }
    else
    {
        double sub_dist = fabs((PathPlanningAlgNew::Distance(&quadPos, &subTarget)));
        if (dist < threshold)
        {
            inSubPath = 0;
            targetObjPos_pub.publish(targetPosNext);
        }
        else if (sub_dist < threshold)
        {
            publishSubTarget(targetObjPos_pub);
            //std::cout << "subTarget Published!" << std::endl;
        }
    }
}


/// < other functions 

/// compute before planning all the nodes that are engaged with me (this will be considered as obstacles during the planning)

bool computeEngagedNodes()
{
    bool res = false;

    setEngagedNodes.clear();

    // lock the mutex for accessing vec_robot_state
    boost::unique_lock<boost::mutex> scoped_lock(vec_robot_state_mutex);

    // compute my distance from all the other robots current position and next position 
    int iNumRobots = vec_robot_state.size();
    for (int id = 0; id < iNumRobots; id++)
    {
        if ((id == my_robot_id) || (vec_robot_state[id].planState.robot_id == -1)) continue;

        vec_robot_state[id].bEngaged = false; // reset the engaged flag 

        int node_start = vec_robot_state[id].planState.currNode;
        int node_end = vec_robot_state[id].planState.nextNode;

        geometry_msgs::PoseStamped startPose;
        geometry_msgs::PoseStamped endPose;

        startPose.pose.position.x = myLRTA.getCoord('x', node_start) * MAP_SCALE - OFS_X; /// The constant is added due to the
        startPose.pose.position.y = myLRTA.getCoord('y', node_start) * MAP_SCALE - OFS_Y; /// different origin of the GRF used in Vrep
        startPose.pose.position.z = vec_robot_state[id].planState.height;

        endPose.pose.position.x = myLRTA.getCoord('x', node_end) * MAP_SCALE - OFS_X; /// The constant is added due to the
        endPose.pose.position.y = myLRTA.getCoord('y', node_end) * MAP_SCALE - OFS_Y; /// different origin of the GRF used in Vrep
        endPose.pose.position.z = vec_robot_state[id].planState.height;

        float distStart = fabs((PathPlanningAlgNew::Distance(&quadPos, &startPose)));
        float distEnd = fabs((PathPlanningAlgNew::Distance(&quadPos, &endPose)));

        const float distCritical = 2 * MAP_SCALE;
        bool bStartCritical = distStart < distCritical;
        bool bEndCritical = distEnd < distCritical;
        if (bStartCritical || bEndCritical)
        {
            vec_robot_state[id].bEngaged = true;

            if (bStartCritical) setEngagedNodes.insert(node_start);
            if (bEndCritical) setEngagedNodes.insert(node_end);

            if (my_robot_id > id) // I stop if I have a smaller robot id
            {
                res = true; // I should stop
                LogRobot << "I'm in node " << vec_robot_state[my_robot_id].planState.currNode << ", engaged with robot " << id //<< ", I should stop!!!" << endl;  
                        << " whose (start,end) = (" << node_start << ", " << node_end << ")" << endl;

                std::set<int> setNeighboursStart;
                std::set<int> setNeighboursEnd;
                if (bStartCritical)
                {
                    myLRTA.getNeighboursSet(node_start, setNeighboursStart);
                    //cout << "neighbours of : " << node_start << endl; 
                    //print(setNeighboursStart);
                }
                if (bEndCritical)
                {
                    myLRTA.getNeighboursSet(node_end, setNeighboursEnd);
                    //cout << "neighbours of : " << node_end << endl; 
                    //print(setNeighboursEnd);
                }
                // compute union between sets and put it inside setNeighboursStart
                setNeighboursStart.insert(setNeighboursEnd.begin(), setNeighboursEnd.end());
                // add the resulting set to the list of engaged nodes 
                for (std::set<int>::iterator it = setNeighboursStart.begin(); it != setNeighboursStart.end(); ++it)
                {
                    if ((*it) != vec_robot_state[my_robot_id].planState.currNode)
                    {
                        setEngagedNodes.insert(*it);
                    }
                }
            }
        }

        LogRobot << " list of engaged nodes: " << endl;
        print(setEngagedNodes);
        cout << endl;
    }

    return res;
}


// identify my robot type from string 
RobotType getRobotType(const std::string& type_str)
{
    //cout << "my robot type str: " << type_str << endl; 
    for (int i = 0; i < kRobotTypeNums; i++)
    {
        if (RobotTypeStr[i] == type_str)
            return (RobotType) i;
    }
    return kRobotTypeNone;
}

// get parameter from hidden node 
template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("[NLA] Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("[NLA] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}

void quadPosFromVrep(const geometry_msgs::PoseStamped::ConstPtr& pubQuadPose)
{
    quadPos.header.frame_id = pubQuadPose->header.frame_id;
    quadPos.header.stamp = pubQuadPose->header.stamp;
    quadPos.pose.position.x = pubQuadPose->pose.position.x;
    quadPos.pose.position.y = pubQuadPose->pose.position.y;
    quadPos.pose.position.z = pubQuadPose->pose.position.z;
    quadPos.pose.orientation.x = pubQuadPose->pose.orientation.x;
    quadPos.pose.orientation.y = pubQuadPose->pose.orientation.y;
    quadPos.pose.orientation.z = pubQuadPose->pose.orientation.z;
    quadPos.pose.orientation.w = pubQuadPose->pose.orientation.w;
    quadPosAcquired = 1;
    //std::cout << "Position of the robot" << my_robot_id << "ACQUIRED!" << std::endl;
}

// get planned target from my team
void updateCount(const quadcopter_ctrl::LRTAmsg::ConstPtr& LRTAinfo)
{
    //LogRobotDebug  << "getting info about robot " << LRTAinfo->robot_id <<" (start,end): (" << LRTAinfo->currNode << ", " << LRTAinfo->nextNode <<")"<< endl; 

    bool printDebug = false;
    myLRTA.incrCount(LRTAinfo->currNode, LRTAinfo->nextNode, LRTAinfo->isNextVisited, printDebug);

    // lock the mutex for accessing vec_robot_state
    boost::unique_lock<boost::mutex> scoped_lock(vec_robot_state_mutex);

    int current_vec_size = vec_robot_state.size();
    if (current_vec_size < (LRTAinfo->robot_id + 1)) // recall that robot_id starts from 0 to (#robots -1)
    {
        vec_robot_state.resize(LRTAinfo->robot_id + 1);
        //cout << "resizing robot state vec, now size " << vec_robot_state.size() << endl; 
    }
    vec_robot_state[LRTAinfo->robot_id].planState = *LRTAinfo;

}

std::string get_selfpath()
{
    char buff[2048];
    ssize_t len = ::readlink("/proc/self/exe", buff, sizeof (buff) - 1);
    if (len != -1)
    {
        buff[len] = '\0';
        std::string path(buff); ///Here the executable name is still in
        std::string::size_type t = path.find_last_of("/"); // Here we find the last "/"
        path = path.substr(0, t); // and remove the rest (exe name)
        return path;
    }
    else
    {
        printf("Cannot determine file path!\n");
    }
}

std::string add_argv(std::string str, char* argvalue)
{

    std::string suffix(argvalue);
    str = str + "_" + suffix;

    return str;

}

// publish my target and other info about myself
void publishTarget(ros::Publisher& countPub)
{
    //targetPos.header.stamp = ros::Time::now();
    //targetPos.header.frame_id = global_frame_id;
    targetPos.header.frame_id = global_frame_id;
    targetPos.pose.position.x = myLRTA.getCurrentCoord('x') * MAP_SCALE - OFS_X; /// The constant is added due to the
    targetPos.pose.position.y = myLRTA.getCurrentCoord('y') * MAP_SCALE - OFS_Y; /// different origin of the GRF used in Vrep
    targetPos.pose.position.z = zHeight;

    targetPosNext.pose.position.x = myLRTA.getNextCoord('x') * MAP_SCALE - OFS_X; /// The constant is added due to the
    targetPosNext.pose.position.y = myLRTA.getNextCoord('y') * MAP_SCALE - OFS_Y; /// different origin of the GRF used in Vrep
    targetPosNext.pose.position.z = zHeight;

    LRTAinfo.robot_id = my_robot_id;
    LRTAinfo.robot_type = my_robot_type;
    LRTAinfo.currNode = myLRTA.getCurrentIndex();
    LRTAinfo.nextNode = myLRTA.getNextIndex();
    LRTAinfo.isNextVisited = myLRTA.getNextType();
    LRTAinfo.height = zHeight;

    //std::cout << "void publishTarget: robot " << my_robot_id << std::endl;
    //std::cout << "myLRTA.getCurrentCoord('x')*MAP_SCALE - OFS_X " << myLRTA.getCurrentCoord('x')*MAP_SCALE - OFS_X << std::endl;
    //std::cout << "myLRTA.getCurrentCoord('y')*MAP_SCALE - OFS_Y " << myLRTA.getCurrentCoord('y')*MAP_SCALE - OFS_Y << std::endl;
    //std::cout << "zHeight " << zHeight << std::endl;
    //std::cout << "myLRTA.getCurrentIndex() " << myLRTA.getCurrentIndex() << std::endl;
    //std::cout << "myLRTA.getNextIndex() " << myLRTA.getNextIndex() << std::endl;
    //std::cout << "myLRTA.getNextType() " << myLRTA.getNextType() << std::endl;

    countPub.publish(LRTAinfo);
}

void publishSubTarget(ros::Publisher& posPub)
{
    double dSubWP[3] = {WP_STEP, WP_STEP, WP_STEP};
    PathPlanningAlgNew::InterpNewPoint(&quadPos, &targetPosNext, dSubWP);
    //subTarget.header.stamp = quadPos.header.stamp;
    //subTarget.header.frame_id = quadPos.header.frame_id;
    subTarget.pose.position.x = quadPos.pose.position.x + dSubWP[X];
    subTarget.pose.position.y = quadPos.pose.position.y + dSubWP[Y];
    subTarget.pose.position.z = quadPos.pose.position.z + dSubWP[Z];
    /*
    std::cout << "publishSubTarget Function" << std::endl;  
    std::cout << "quadPos.pose.position.x " << quadPos.pose.position.x << std::endl;
    std::cout << "quadPos.pose.position.y " << quadPos.pose.position.y << std::endl;
    std::cout << "quadPos.pose.position.z " << quadPos.pose.position.z << std::endl;
    std::cout << "dSubWP[X] " << dSubWP[X] << std::endl;
    std::cout << "dSubWP[Y] " << dSubWP[Y] << std::endl;
    std::cout << "dSubWP[Z] " << dSubWP[Z] << std::endl;
     */
    posPub.publish(subTarget);
}

void print(std::set<int>& setin)
{
    for (std::set<int>::iterator it = setin.begin(); it != setin.end(); ++it)
    {
        if (it != setin.begin()) cout << ", ";
        cout << *it;
    }
    cout << endl;
}
