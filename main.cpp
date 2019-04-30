#include <vector>
#include <fstream>
#include <math.h>
#include <ros/console.h>
#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/base/Matrix.h>

/** ros include */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

/** local include */
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "colocalization/optimizeFactorGraph.h"
#include "colocalization/addBearingRangeNodes.h"
#include "bearing_estimator/estimate_bearing.h"
#include "bearing_estimator/estimate_range.h"
#include "bearing_estimator/ground_truth_range.h"
#include "bearing_estimator/ground_truth_bearing.h"
#include "tf/transform_datatypes.h"
#include "piksi_rtk_msgs/BaselineNed.h"
// #include "LinearMath/btMatrix3x3.h"

using namespace std;
using namespace gtsam;

/*  Localization Node
    - Running on base station
    - Subscribes
      - Odometry from both rovers
        - Adds odometry data to the factorgraph
    - Services
      - Call services
         - Range from rover having anchor decawave
            - Adds range data between two nodes to the factorgraph
         - Bearing from both rovers
            - Adds bearing data between two nodes to the factorgraph.Direction of bearing is important here
      - Provides services
         - add bearing and range nodes in factor graph
            - Calls range and bearing service and adds the nodes in the graph and replies back with success or not.
         - optimize pose of both rovers
            - Solves batch factor graph and replies back with updated pose.
    - Publishes
      - None
*/
// This variable is not requried
// Service client to request bearing of rover 2 from rover 1
ros::ServiceClient bearingClient12;

// Service client to request bearing of rover 2 from rover 1
ros::ServiceClient bearingClient21;
ros::ServiceClient trueBearingClient;

// Service client to request relative range between two rovers.
ros::ServiceClient rangeClient;
ros::ServiceClient trueRangeClient;

// TODO: [Stretch] Make the code modular enough to support multiple rovers.


ros::Publisher pose1_pub;
ros::Publisher pose2_pub;
ros::Publisher odom_error_pub;
ros::Publisher colocalization_error_pub;

NonlinearFactorGraph newFactors = NonlinearFactorGraph();
gtsam::Values newValues = Values();
gtsam::Values realValues = Values();

bool ak1_init = 0;
bool ak2_init = 0;
int ak1_factor_nodes_count = 0;
int ak2_factor_nodes_count = 0;
double odometryRSigma = 0.1;
double odometryThetaSigma = 0.025;
double measuredBearingNoiseSigma = 0.1;
double measuredRangeNoiseSigma = 0.1;

noiseModel::Diagonal::shared_ptr priorNoise1 = noiseModel::Diagonal::Sigmas(Vector3(0.000001, 0.000001, 0.000001));
noiseModel::Diagonal::shared_ptr priorNoise2 = priorNoise1;

noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(odometryRSigma, odometryRSigma, odometryThetaSigma));

noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2));
noiseModel::Diagonal::shared_ptr bearingNoise = noiseModel::Diagonal::Sigmas((Vector(1) << measuredBearingNoiseSigma));

noiseModel::Diagonal::shared_ptr rangeNoise = noiseModel::Diagonal::Sigmas((Vector(1) << measuredRangeNoiseSigma));

Pose2 ak1_cur_pose;
Symbol last_ak1_symbol;
Pose2 ak2_cur_pose;
Symbol last_ak2_symbol;

gtsam::Pose2 last_pose1 = gtsam::Pose2(0,0,0);
gtsam::Pose2 last_pose2 = gtsam::Pose2(0,0,0);

Eigen::Matrix3d H_rover22rtk;

// TODO: Class for Odometry

/**
 *
 */
float distance(gtsam::Pose2 current_pose, gtsam::Pose2 last_pose)
{
    float dist = std::sqrt(pow(current_pose.x() - last_pose.x(), 2) + pow(current_pose.y()-last_pose.y(), 2));
    // cout << "Distance from previous odom measurement: " << dist << endl;
    return dist;
}

/**
 *
 */
float get_yaw(geometry_msgs::Quaternion orientation){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation, quat);
    quat.normalize();
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

/**
 *
 */
gtsam::Pose2 get_pose_change_robot_frame(gtsam::Pose2 pose, gtsam::Pose2 last_pose){
    double dx = pose.x() - last_pose.x();
    double dy = pose.y() - last_pose.y();
    double dtheta = pose.theta() - last_pose.theta();

    double yaw = pose.theta();
    double dxr = dx*cos(-yaw) - dy*sin(-yaw);
    double dyr = dx*sin(-yaw) + dy*cos(-yaw);
    double dthr = dtheta;

    gtsam::Pose2 ak2_change = gtsam::Pose2(dxr, dyr, dthr);
    return ak2_change;
}

/**
 *
 */
void odometry1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    cout << "Odometry1 called" << endl;
    geometry_msgs::Pose pose = msg->pose.pose;

    float yaw = get_yaw(pose.orientation);
    gtsam::Pose2 current_pose = gtsam::Pose2(pose.position.x, pose.position.y, yaw);
    gtsam::Pose2 ak1_change = get_pose_change_robot_frame(current_pose, last_pose1);
    gtsam::Symbol symbol = gtsam::Symbol('a', ak1_factor_nodes_count++);
    float dist = distance(current_pose, last_pose1);
    cout << "Distance from previous odom measurement: " << dist << endl;
    if (dist>=0.2) {
        if(!ak1_init)
        {
            newFactors.add(PriorFactor<Pose2>(symbol, current_pose, priorNoise1));
            newValues.insert(symbol, current_pose);
            ak1_cur_pose = current_pose;
            ak1_init = true;
        }
        else
        {
            newFactors.add(BetweenFactor<Pose2>(last_ak1_symbol, symbol, ak1_change, odometryNoise));
            ak1_cur_pose = ak1_cur_pose.compose(ak1_change);
            newValues.insert(symbol, ak1_cur_pose);
        }
        last_ak1_symbol = symbol;
        last_pose1 = current_pose;
    }
    cout << "Odometry1 calculated"  << endl;
    // piksi_rtk_msgs::BaselineNedConstPtr real_pose = ros::topic::waitForMessage<piksi_rtk_msgs::BaselineNed>("/ak2/piksi/baseline_ned");
    // gtsam::Pose2 current_real_pose = gtsam::Pose2(real_pose->pose.position.x, real_pose->pose.position.y, 0);
    // realValues.insert(symbol, current_real_pose);
}

/**
 *
 */
void odometry2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // cout << "Odometry2 called" << endl;
    cout << ak2_factor_nodes_count << endl;
    geometry_msgs::Pose pose = msg->pose.pose;
    float yaw = get_yaw(pose.orientation);
    gtsam::Pose2 current_pose = gtsam::Pose2(pose.position.x, pose.position.y, yaw);
    gtsam::Pose2 ak2_change = get_pose_change_robot_frame(current_pose, last_pose2);
    gtsam::Symbol symbol = gtsam::Symbol('b', ak2_factor_nodes_count++);
    float dist = distance(current_pose, last_pose2);
    if(dist >= 0.2){
        cout << "Distance from previous odom measurement: " << dist << endl;
        if(!ak2_init)
        {

            newFactors.add(PriorFactor<Pose2>(symbol, current_pose, priorNoise2));
            newValues.insert(symbol, current_pose);
            ak2_cur_pose = current_pose;
            ak2_init = true;
        }
        else
        {
            newFactors.add(BetweenFactor<Pose2>(last_ak2_symbol, symbol, ak2_change, odometryNoise));
            ak2_cur_pose= ak2_cur_pose.compose(ak2_change);
            newValues.insert(symbol, ak2_cur_pose);
        }
        last_ak2_symbol = symbol;
        last_pose2 = current_pose;
    }
    piksi_rtk_msgs::BaselineNedConstPtr real_pose = ros::topic::waitForMessage<piksi_rtk_msgs::BaselineNed>("/ak2/piksi/baseline_ned");
    gtsam::Pose2 current_real_pose = gtsam::Pose2(real_pose->e/1000.0, real_pose->n/1000.0, 0);
    realValues.insert(symbol, current_real_pose);

}

/**
 *
 */
bool addBearingRangeNodes(colocalization::addBearingRangeNodes::Request& request, colocalization::addBearingRangeNodes::Response &response)
{

    cout << "addBearingRangeNodes called" << endl;

    bool odom1_added = false;
    // Adding odom1
    gtsam::Symbol symbol_ak1;
    boost::shared_ptr<nav_msgs::Odometry const> sharedPtr1;
    sharedPtr1 = ros::topic::waitForMessage<nav_msgs::Odometry>("/ak1/odom", ros::Duration(10));
    if(sharedPtr1 == NULL)
        ROS_WARN("No odometry received for rover1");
    else
    {
        nav_msgs::Odometry odom1;
        odom1 = *sharedPtr1;
        geometry_msgs::Pose pose1 = odom1.pose.pose;
        float yaw1 = get_yaw(pose1.orientation);
        gtsam::Pose2 current_pose1 =  gtsam::Pose2(pose1.position.x, pose1.position.y, yaw1);
        gtsam::Pose2 ak1_change = get_pose_change_robot_frame(current_pose1, last_pose1);
        symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count++);
        newFactors.add(BetweenFactor<Pose2>(last_ak1_symbol, symbol_ak1, ak1_change, odometryNoise));
        ak1_cur_pose = ak1_cur_pose.compose(ak1_change);
        newValues.insert(symbol_ak1, ak1_cur_pose);
        last_ak1_symbol = symbol_ak1;
        last_pose1 = current_pose1;
        odom1_added = true;
        ROS_INFO("Odometry 1 added successfully");
    }

    bool odom2_added = false;
    boost::shared_ptr<nav_msgs::Odometry const> sharedPtr2;
    sharedPtr2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/ak2/odom", ros::Duration(10));
    gtsam::Symbol symbol_ak2;
    if(sharedPtr2 == NULL)
        ROS_WARN("No odometry received for rover2");
    else
    {
        nav_msgs::Odometry odom2;
        odom2 = *sharedPtr1;
        geometry_msgs::Pose pose2 = odom2.pose.pose;
        float yaw2 = get_yaw(pose2.orientation);
        gtsam::Pose2 current_pose2 =  gtsam::Pose2(pose2.position.x, pose2.position.y, yaw2);
        gtsam::Pose2 ak2_change = get_pose_change_robot_frame(current_pose2, last_pose2);

        symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count++);
        newFactors.add(BetweenFactor<Pose2>(last_ak2_symbol, symbol_ak2, ak2_change, odometryNoise));
        ak2_cur_pose = ak2_cur_pose.compose(ak2_change);
        newValues.insert(symbol_ak2, ak2_cur_pose);
        last_ak2_symbol = symbol_ak2;
        last_pose2 = current_pose2;
        odom2_added = true;
        ROS_INFO("Odometry 2 added successfully");
    }


    if(odom1_added && odom2_added)
    {
        bearing_estimator::estimate_bearing bearing12_srv;
        if (ros::service::exists("ak1/estimate_bearing", true) && bearingClient12.call(bearing12_srv))
        {
            newFactors.add(BearingFactor<Pose2, Pose2>(symbol_ak1, symbol_ak2, Rot2(bearing12_srv.response.bearing.bearing), bearingNoise));
            ROS_INFO("Added bearing12");
        }

        // Add bearing measurement from rover 2 to rover 1
        bearing_estimator::estimate_bearing bearing21_srv;
        if (ros::service::exists("ak2/estimate_bearing", true) && bearingClient21.call(bearing21_srv))
        {
            newFactors.add(BearingFactor<Pose2, Pose2>(symbol_ak2, symbol_ak1, Rot2(bearing21_srv.response.bearing.bearing), bearingNoise));
            ROS_INFO("Added bearing21");
        }

        // Add range measurement from rover with anchor sensor
        bearing_estimator::estimate_range estimate_range_srv;
        if (rangeClient.call(estimate_range_srv))
        {
            newFactors.add(RangeFactor<Pose2, Pose2>(symbol_ak2, symbol_ak1, estimate_range_srv.response.range.range, rangeNoise));
            ROS_INFO("Added range");
        }

        bearing_estimator::ground_truth_range true_range_srv;
        trueRangeClient.call(true_range_srv);

        // bearing_estimator::ground_truth_bearing true_bearing_srv;
        // trueBearingClient.call(true_bearing_srv);
    }
    // newFactors.print(" Factor Graph");
    return true;
}

float calculate_error_metric(gtsam::Values estimated_path, gtsam::Values real_path, int rover_n)
{
    int nodes_n = rover_n == 1 ? ak1_factor_nodes_count : ak2_factor_nodes_count;
    char rover_letter = rover_n == 1 ? 'a' : 'b';

    float error = 0;

    for(int i=0; i<nodes_n;i++)
    {
        gtsam::Symbol symbol = gtsam::Symbol(rover_letter, i);
        gtsam::Pose2* estimated_pose = (gtsam::Pose2*) &estimated_path.at(symbol);
        gtsam::Pose2* real_pose = (gtsam::Pose2*) &real_path.at(symbol);

        error += std::sqrt(pow(real_pose->x() - estimated_pose->x(), 2) + pow(real_pose->y()-estimated_pose->y(), 2));
    }

    return error;
}

float calculateRealDistanceTravelled(gtsam::Values real_path, int rover_n)
{
    int nodes_n = rover_n == 1 ? ak1_factor_nodes_count : ak2_factor_nodes_count;
    char rover_letter = rover_n == 1 ? 'a' : 'b';
    float distance = 0;
    for(int i=1; i<nodes_n;i++)
    {
        gtsam::Symbol current_symbol = gtsam::Symbol(rover_letter, i);
        gtsam::Symbol previous_symbol = gtsam::Symbol(rover_letter, i-1);
        if(real_path.exists(current_symbol) && real_path.exists(previous_symbol))
        {
            gtsam::Pose2* current_pose = (gtsam::Pose2*) &real_path.at(current_symbol);
            gtsam::Pose2* previous_pose = (gtsam::Pose2*) &real_path.at(previous_symbol);
            distance += std::sqrt(pow(current_pose->x() - previous_pose->x(), 2) + pow(current_pose->y()-previous_pose->y(), 2));
        }
    }
    return distance;
}

float calculateDrift(gtsam::Values real_path, gtsam::Values estimated_path, int rover_n)
{
    bool last_node_found = 0;
    float drift;
    int nodes_n = rover_n == 1 ? ak1_factor_nodes_count : ak2_factor_nodes_count;
    char rover_letter = rover_n == 1 ? 'a' : 'b';
    while(!last_node_found)
    {
        gtsam::Symbol symbol = gtsam::Symbol(rover_letter, nodes_n);
        if(real_path.exists(symbol) && estimated_path.exists(symbol))
        {
            last_node_found = 1;
            gtsam::Pose2* real_pose = (gtsam::Pose2*) &real_path.at(symbol);
            gtsam::Pose2* estimated_pose = (gtsam::Pose2*) &estimated_path.at(symbol);
            drift = std::sqrt(pow(real_pose->x() - estimated_pose->x(), 2) + pow(real_pose->y()-estimated_pose->y(), 2));
        }
        nodes_n -= 1;
    }
    return drift;
}
/**
 *
 */
bool optimizeFactorGraph(colocalization::optimizeFactorGraph::Request& request, colocalization::optimizeFactorGraph::Response &response)
{
    cout << "optimizeFactorGraph called" << endl;
    newValues.print("Odometry Result:\n");

    gtsam::LevenbergMarquardtParams LMParams;
    LMParams.setLinearSolverType("MULTIFRONTAL_QR");
    gtsam::LevenbergMarquardtOptimizer optimizer = gtsam::LevenbergMarquardtOptimizer(newFactors, newValues, LMParams);

    gtsam::Values result = optimizer.optimize();
    size_t size = result.size();
    result.print("Final Result:\n");

    response.size = size;
    // float odom_drift1, colocalize_drift1, distance_travelled1;
    // for(int i=0; i<ak1_factor_nodes_count;i++)
    // {
    //     if(i<realValues.size() && i<newValues.size() && i<result.size())
    //     {
    //         gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', i);
    //         gtsam::Pose2* pose1 = (gtsam::Pose2*) &result.at(symbol_ak1);

    //         geometry_msgs::Point position;
    //         position.x = pose1->x();
    //         position.y = pose1->y();

    //         geometry_msgs::Quaternion orientation;
    //         orientation.z = pose1->theta();

    //         geometry_msgs::Pose pose;
    //         pose.position = position;
    //         pose.orientation = orientation;

    //         response.poses1.poses.push_back(pose);
    //         distance_travelled1 = calculateRealDistanceTravelled(realValues, 1);
    //         odom_drift1 = calculateDrift(realValues, newValues, 1);
    //         colocalize_drift1 = calculateDrift(realValues, result, 1);
    //     }
    // }
    // geometry_msgs::PoseArray poses1 = response.poses1;
    // pose1_pub.publish(poses1);


    float odom_drift2, colocalize_drift2, distance_travelled2;
    for(int i=0; i<ak2_factor_nodes_count;i++)
    {
        if(i<realValues.size() && i<newValues.size() && i<result.size())
        {
            gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', i);
            gtsam::Pose2* pose2 = (gtsam::Pose2*) &result.at(symbol_ak2);

            Eigen::Vector3d p(pose2->x(), pose2->y(), 1);
            Eigen::Vector3d transformed_pose = H_rover22rtk*p;

            geometry_msgs::Point position;
            position.x = transformed_pose[0];
            position.y = transformed_pose[1];

            geometry_msgs::Quaternion orientation;
            orientation.z = pose2->theta();

            geometry_msgs::Pose pose;
            pose.position = position;
            pose.orientation = orientation;

            response.poses2.poses.push_back(pose);
        }
    }
    distance_travelled2 = calculateRealDistanceTravelled(realValues, 2);
    odom_drift2 = calculateDrift(realValues, newValues, 2);
    colocalize_drift2 = calculateDrift(realValues, result, 2);

    geometry_msgs::PoseArray poses2 = response.poses2;
    pose2_pub.publish(poses2);


    // std_msgs::Float32 odom_error;
    // odom_error.data = (odom_drift2/distance_travelled2 + odom_drift1/distance_travelled1)/2;
    // std_msgs::Float32 colocalization_error;
    // colocalization_error.data = (colocalize_drift2/distance_travelled2 + colocalize_drift1/distance_travelled1)/2;

    std_msgs::Float32 odom_error;
    odom_error.data = (odom_drift2/distance_travelled2);
    odom_error_pub.publish(odom_error);
    cout << odom_error.data << endl;
    std_msgs::Float32 colocalization_error;
    colocalization_error.data = (colocalize_drift2/distance_travelled2);
    colocalization_error_pub.publish(colocalization_error);
    cout << colocalization_error.data << endl;

    return true;
}

void set_transformation(piksi_rtk_msgs::BaselineNedConstPtr initial_pose)
{
    cout << "Transformation set" << endl;
    H_rover22rtk << 0, 0, initial_pose->e/1000.0,
        0, 0, initial_pose->n/1000.0,
        0, 0, 1;
    cout << H_rover22rtk << endl;
}

/**
 *
 */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "colocalization");
    ros::NodeHandle n;
    cout << "Started" << endl;

    // Set Odometry and RTK GPS transformation
    piksi_rtk_msgs::BaselineNedConstPtr initial_pose = ros::topic::waitForMessage<piksi_rtk_msgs::BaselineNed>("/ak2/piksi/baseline_ned");
    set_transformation(initial_pose);

    // TODO: We can use one odometry callback by having a Class for "Odometry"
    ros::Subscriber odometry1_sub = n.subscribe("/ak1/odom", 1000, odometry1Callback);
    ros::Subscriber odometry2_sub = n.subscribe("/ak2/odom", 1000, odometry2Callback);

    // Updated Pose Publisher
    pose1_pub = n.advertise<geometry_msgs::PoseArray>("/ak1/pose1", 1000);
    pose2_pub = n.advertise<geometry_msgs::PoseArray>("/ak2/pose2", 1000);

    // Error metric Publisher
    odom_error_pub = n.advertise<std_msgs::Float32>("/odom_error", 1000);
    colocalization_error_pub = n.advertise<std_msgs::Float32>("/colocalize_error", 1000);

    // Provided services
    ros::ServiceServer addBearingRangeNodesService = n.advertiseService("addBearingRangeNodes", addBearingRangeNodes);
    ros::ServiceServer optimizeFactorGraphService = n.advertiseService("optimizeFactorGraph", optimizeFactorGraph);

    // Subscribed estimated services
    bearingClient12 = n.serviceClient<bearing_estimator::estimate_bearing>("/ak1/estimate_bearing");
    bearingClient21 = n.serviceClient<bearing_estimator::estimate_bearing>("/ak2/estimate_bearing");
    rangeClient = n.serviceClient<bearing_estimator::estimate_range>("estimate_range");

    // Subscribed ground truth services
    trueRangeClient = n.serviceClient<bearing_estimator::ground_truth_range>("ground_truth_range");
    // trueBearingClient = n.serviceClient<bearing_estimator::ground_truth_bearing>("ground_truth_bearing");

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spin();
        loop_rate.sleep();
    }
}
