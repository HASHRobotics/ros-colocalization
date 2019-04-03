#include <vector>
#include <fstream>
#include <math.h>

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

/** ros include */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/** local include */
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "colocalization/optimizeFactorGraph.h"
#include "colocalization/addBearingRangeNodes.h"
#include "bearing_estimator/estimate_bearing.h"
#include "bearing_estimator/get_range.h"
#include "tf/transform_datatypes.h"
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
int i = 0;

ros::NodeHandle n;

NonlinearFactorGraph newFactors = NonlinearFactorGraph();
gtsam::Values newValues = Values();
ros::Publisher pose_pub1;
ros::Publisher pose_pub2;

bool ak1_init = 0;
bool ak2_init = 0;
int ak1_factor_nodes_count = 0;
int ak2_factor_nodes_count = 0;
double odometryRSigma = 0.05;
double odometryThetaSigma = 0.025;
double measuredBearingNoiseSigma = 0.1;
double measuredRangeNoiseSigma = 0.1;

noiseModel::Diagonal::shared_ptr priorNoise1 = noiseModel::Diagonal::Sigmas(Vector3(0.000001, 0.000001, 0.000001));
noiseModel::Diagonal::shared_ptr priorNoise2 = priorNoise1;

noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(odometryRSigma, odometryRSigma, odometryThetaSigma));

noiseModel::Diagonal::shared_ptr bearingNoise = noiseModel::Diagonal::Sigmas(Vector3(measuredBearingNoiseSigma));

noiseModel::Diagonal::shared_ptr rangeNoise = noiseModel::Diagonal::Sigmas(Vector3(measuredRangeNoiseSigma));

Pose2 ak1_cur_pose;
Symbol last_ak1_symbol;
Pose2 ak2_cur_pose;
Symbol last_ak2_symbol;

gtsam::Pose2 last_pose1 = gtsam::Pose2(0,0,0);
gtsam::Pose2 last_pose2 = gtsam::Pose2(0,0,0);

float distance(gtsam::Pose2 current_pose, gtsam::Pose2 last_pose)
{
    return std::sqrt(pow(current_pose.x() - last_pose.x(), 2) + pow(current_pose.y()-last_pose.y(), 2));
}

void odometry1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    // Check how to convert quaternion to theta
    gtsam::Pose2 current_pose = gtsam::Pose2(pose.position.x - last_pose1.x(), pose.position.y - last_pose1.y(), pose.orientation.z - last_pose1.theta());
    float dist = distance(current_pose, last_pose1);
    if (dist>1) {
        gtsam::Symbol symbol = gtsam::Symbol('a', ak1_factor_nodes_count++);
        if(!ak1_init)
        {
            newFactors.add(PriorFactor<Pose2>(symbol, current_pose, priorNoise1));
            newValues.insert(symbol, current_pose);
            ak1_cur_pose = current_pose;
            ak1_init = true;
        }
        else
        {
            newFactors.add(BetweenFactor<Pose2>(last_ak1_symbol, symbol, current_pose, odometryNoise));
            ak1_cur_pose.compose(current_pose);
            newValues.insert(symbol, ak1_cur_pose);
        }
        last_ak1_symbol = symbol;
        last_pose1 = current_pose;
    }
}

float get_yaw(geometry_msgs::Quaternion orientation){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

void odometry2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    float yaw = get_yaw(pose.orientation);
    gtsam::Pose2 current_pose = gtsam::Pose2(pose.position.x, pose.position.y, yaw);

    double dx = pose.position.x - last_pose2.x();
    double dy = pose.position.y - last_pose2.y();
    double dtheta = yaw - last_pose2.theta();

    double dxr = dx*cos(-yaw) - dy*sin(-yaw);
    double dyr = dx*sin(-yaw) + dy*cos(-yaw);
    double dthr = dtheta;

    gtsam::Pose2 ak2_change = gtsam::Pose2(dxr, dyr, dthr);
    float dist = distance(current_pose, last_pose2);
    if(dist > 1){
        gtsam::Symbol symbol = gtsam::Symbol('b', ak2_factor_nodes_count++);
        if(!ak2_init)
        {
            newFactors.add(PriorFactor<Pose2>(symbol, ak2_change, priorNoise2));
            newValues.insert(symbol, current_pose);
            ak2_cur_pose = current_pose;
            ak2_init = true;
        }
        else
        {
            newFactors.add(BetweenFactor<Pose2>(last_ak2_symbol, symbol, ak2_change, odometryNoise));
            ak2_cur_pose.compose(current_pose);
            newValues.insert(symbol, ak2_cur_pose);
        }
        last_ak2_symbol = symbol;
        last_pose2 = current_pose;
    }
}

// Change to localization request response
bool addBearingRangeNodes(colocalization::addBearingRangeNodes::Request& request, colocalization::addBearingRangeNodes::Response &response)
{
    // Adding current odometry Node for rover 1
    nav_msgs::OdometryConstPtr odom1 = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom1");
    geometry_msgs::Pose pose1 = odom1->pose.pose;
    gtsam::Pose2 current_pose1 = gtsam::Pose2(pose1.position.x - last_pose1.x(), pose1.position.y - last_pose1.y(), pose1.orientation.z - last_pose1.theta());
    gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count++);
    newFactors.add(BetweenFactor<Pose2>(last_ak1_symbol, symbol_ak1, current_pose1, odometryNoise));
    ak1_cur_pose.compose(current_pose1);
    newValues.insert(symbol_ak1, ak1_cur_pose);
    last_ak1_symbol = symbol_ak1;
    last_pose1 = current_pose1;

    // Adding current odometry Node for rover 2
    nav_msgs::OdometryConstPtr odom2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom2");
    geometry_msgs::Pose pose = odom2->pose.pose;
    gtsam::Pose2 current_pose2 = gtsam::Pose2(pose1.position.x - last_pose2.x(), pose1.position.y - last_pose2.y(), pose1.orientation.z - last_pose2.theta());
    gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count++);
    newFactors.add(BetweenFactor<Pose2>(last_ak2_symbol, symbol_ak2, current_pose2, odometryNoise));
    ak2_cur_pose.compose(current_pose2);
    newValues.insert(symbol_ak2, ak2_cur_pose);
    last_ak2_symbol = symbol_ak2;
    last_pose2 = current_pose2;

    // Add bearing measurement from rover 1 to rover 2
    ros::ServiceClient bearingClient12 = n.serviceClient<bearing_estimator::estimate_bearing>("bearing_estimate12");
    bearing_estimator::estimate_bearing bearing12_srv;
    if (bearingClient12.call(bearing12_srv))
    {
        newFactors.add(BearingFactor<Pose2, Point2>(symbol_ak1, symbol_ak2, Rot2(bearing12_srv.response.bearing.bearing), bearingNoise));
    }

    // Add bearing measurement from rover 2 to rover 1
    ros::ServiceClient bearingClient21 = n.serviceClient<bearing_estimator::estimate_bearing>("bearing_estimate21");
    bearing_estimator::estimate_bearing bearing21_srv;
    if (bearingClient21.call(bearing21_srv))
    {
        newFactors.add(BearingFactor<Pose2, Point2>(symbol_ak2, symbol_ak1, Rot2(bearing21_srv.response.bearing.bearing), bearingNoise));
    }

    // Add range measurement from rover with anchor sensor
    ros::ServiceClient rangeClient = n.serviceClient<bearing_estimator::get_range>("range_estimate");
    bearing_estimator::get_range range_srv;
    if (rangeClient.call(range_srv))
    {
        newFactors.add(RangeFactor<Pose2, Point2>(symbol_ak2, symbol_ak1, range_srv.response.range.range, rangeNoise));
        newFactors.add(RangeFactor<Pose2, Point2>(symbol_ak1, symbol_ak2, range_srv.response.range.range, rangeNoise));
    }
    return true;
}

bool optimizeFactorGraph(colocalization::optimizeFactorGraph::Request& request, colocalization::optimizeFactorGraph::Response &response)
{
    // Publish updated path data as well.
    int current_ak1_factor_nodes_count = ak1_factor_nodes_count;
    int current_ak2_factor_nodes_count = ak2_factor_nodes_count;
    gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count);
    gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count);

    gtsam::LevenbergMarquardtParams LMParams;
    LMParams.setLinearSolverType("MULTIFRONTAL_QR");
    gtsam::LevenbergMarquardtOptimizer optimizer = gtsam::LevenbergMarquardtOptimizer(newFactors, newValues, LMParams);
    gtsam::Values result = optimizer.optimize();
    size_t size = result.size();
    result.print("Final Result:\n");
    // To Do: Marginals marginals(newFactors, result);
    int count = 0;
    response.size = size;
    for(int i=0; i<ak1_factor_nodes_count;i++)
    {
        gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', i);
        gtsam::Pose2* pose1 = (gtsam::Pose2*) &result.at(symbol_ak1);
        response.poses1.poses[i].position.x = pose1->x();
        response.poses1.poses[i].position.y = pose1->y();
        response.poses1.poses[i].orientation.z = pose1->theta();
    }
    for(int i=0; i<ak2_factor_nodes_count;i++)
    {
        gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', i);
        gtsam::Pose2* pose2 = (gtsam::Pose2*) &result.at(symbol_ak2);
        response.poses2.poses[i].position.x = pose2->x();
        response.poses2.poses[i].position.y = pose2->y();
        response.poses2.poses[i].orientation.z = pose2->theta();
    }
    return true;
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "colocalization");

    ros::Subscriber odometry1_sub = n.subscribe("/odom1", 1000, odometry1Callback);
    ros::Subscriber odometry2_sub = n.subscribe("/odom2", 1000, odometry2Callback);
    ros::ServiceServer addBearingRangeNodesService = n.advertiseService("addBearingRangeNodes", addBearingRangeNodes);
    ros::ServiceServer optimizeFactorGraphService = n.advertiseService("optimizeFactorGraph", optimizeFactorGraph);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose2D>("estimated_pose", 10);
}

// Moved the code to services
// void bearing12Callback(const std_msgs::Float64::ConstPtr& msg)
// {
//     gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count);
//     gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count);
//     newFactors.add(BearingFactor<Pose2, Point2>(symbol_ak1, symbol_ak2, Rot2(msg->data), bearingNoise));
// }

// void bearing21Callback(const std_msgs::Float64::ConstPtr& msg)
// {
//     gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count);
//     gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count);
//     newFactors.add(BearingFactor<Pose2, Point2>(symbol_ak2, symbol_ak1, Rot2(msg->data), bearingNoise));
// }

// void range12Callback(const sensor_msgs::Range::ConstPtr& msg)
// {
//     gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count);
//     gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count);
//     newFactors.add(RangeFactor<Pose2, Point2>(symbol_ak1, symbol_ak2, msg->range, rangeNoise));
// }

// void range21Callback(const sensor_msgs::Range::ConstPtr& msg)
// {

//     gtsam::Symbol symbol_ak2 = gtsam::Symbol('b', ak2_factor_nodes_count);
//     gtsam::Symbol symbol_ak1 = gtsam::Symbol('a', ak1_factor_nodes_count);
//     newFactors.add(RangeFactor<Pose2, Point2>(symbol_ak2, symbol_ak1, msg->range, rangeNoise));
// }
