#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"

#include "vader_msgs/PepperArray.h"
// #include "vader_msgs/BimanualPlanRequest.h"
// #include "vader_msgs/BimanualExecRequest.h"
// #include "vader_msgs/MoveToStorageRequest.h"
#include "vader_msgs/GripperCommand.h"
#include "vader_msgs/CutterCommand.h"

#include "vader_msgs/HarvestResult.h"

#include "vader_msgs/PlanningRequest.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


static void coarseEstimateCallback(const vader_msgs::PepperArray::ConstPtr &msg);
static void fineEstimateCallback(const vader_msgs::PepperArray::ConstPtr &msg);

namespace rvt = rviz_visual_tools;

const double REJECT_FINE_POSE_BEYOND_ANGLE_DEGREES = 20;
const bool SKIP_WORKSPACE_BOUNDS_CHECK = false;

class VADERStateMachine
{
private:
    enum State
    {
        // ----------Coarse-Pregrasp stage-----------
        HomeGripper,
        HomeCutter,
        WaitForCoarseEstimate,
        ParallelMovePregrasp,
        // ----------Fine-Grasp stage-----------
        WaitForFineEstimate,
        GripperGrasp,
        GripperEndEffector,
        // ------------Harvest stage-----------
        CutterGrasp,
        CutterEndEffector,
    // -----------Finish and cleanup-----------
        ParallelMoveStorage,
        HomeGripper2,
        Done,
        Error,
        End
    };

    State currentState;
    ros::NodeHandle n;

    // Perception subscriptions and data
    ros::Subscriber coarseEstimateSub, fineEstimateSub;
    std::vector<vader_msgs::Pepper> coarseEstimates;
    std::vector<vader_msgs::Pepper> fineEstimates;
    vader_msgs::Pepper *coarseEstimate;
    vader_msgs::Pepper *fineEstimate;

    bool allowCoarseEstimateUpdate = true, allowFineEstimateUpdate = true;

    ros::Time waitForFinePoseStartTime;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world"); //shared from planner server main instance

    // Plan/Exec clients connecting to planner
    ros::ServiceClient planClient;

    // End effector talking to dynamixel node
    ros::Publisher gripperCommandPub, cutterCommandPub;

    ros::Publisher resultStatusPub; //publishes to /harvest_result

    /*
    Input: cameraFrameMsg containing the pose of the fruit in the camera frame from the coarse/fine pepper estimate
    Output: result containing the pose of the fruit in the world frame using TF2.
    */
    void _transformFromCameraFrameIntoRobotFrame(const vader_msgs::Pepper cameraFrameMsg, vader_msgs::Pepper *result)
    {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        geometry_msgs::PoseStamped fruit_pose;
        fruit_pose.pose.position.x = cameraFrameMsg.fruit_data.pose.position.x;
        fruit_pose.pose.position.y = cameraFrameMsg.fruit_data.pose.position.y;
        fruit_pose.pose.position.z = cameraFrameMsg.fruit_data.pose.position.z;
        fruit_pose.pose.orientation.x = cameraFrameMsg.fruit_data.pose.orientation.x;
        fruit_pose.pose.orientation.y = cameraFrameMsg.fruit_data.pose.orientation.y;
        fruit_pose.pose.orientation.z = cameraFrameMsg.fruit_data.pose.orientation.z;
        fruit_pose.pose.orientation.w = cameraFrameMsg.fruit_data.pose.orientation.w;
        fruit_pose.header.frame_id = cameraFrameMsg.header.frame_id;
        // ROS_INFO("Original pose: x=%f, y=%f, z=%f, frame_id=%s",
        //           fruit_pose.pose.position.x,
        //           fruit_pose.pose.position.y,
        //           fruit_pose.pose.position.z,
        //           fruit_pose.header.frame_id.c_str());
        bool hasPeduncleData = !(cameraFrameMsg.header.frame_id.empty());
        
        geometry_msgs::PoseStamped peduncle_pose;
        if(hasPeduncleData){
            peduncle_pose.pose.position.x = cameraFrameMsg.peduncle_data.pose.position.x;
            peduncle_pose.pose.position.y = cameraFrameMsg.peduncle_data.pose.position.y;
            peduncle_pose.pose.position.z = cameraFrameMsg.peduncle_data.pose.position.z;
            peduncle_pose.pose.orientation.x = cameraFrameMsg.peduncle_data.pose.orientation.x;
            peduncle_pose.pose.orientation.y = cameraFrameMsg.peduncle_data.pose.orientation.y;
            peduncle_pose.pose.orientation.z = cameraFrameMsg.peduncle_data.pose.orientation.z;
            peduncle_pose.pose.orientation.w = cameraFrameMsg.peduncle_data.pose.orientation.w;
            peduncle_pose.header.frame_id = cameraFrameMsg.header.frame_id;
            // ROS_INFO("Original peduncle pose: x=%f, y=%f, z=%f, frame_id=%s",
            //           peduncle_pose.pose.position.x,
            //           peduncle_pose.pose.position.y,
            //           peduncle_pose.pose.position.z,
            //           peduncle_pose.header.frame_id.c_str());
        }
        

        try
        {
            geometry_msgs::PoseStamped transformed_pose = tf_buffer.transform(
                fruit_pose,
                "world",
                ros::Duration(3.0));
            // ROS_INFO("After transform");
            result->fruit_data.pose.position.x = transformed_pose.pose.position.x;
            result->fruit_data.pose.position.y = transformed_pose.pose.position.y;
            result->fruit_data.pose.position.z = transformed_pose.pose.position.z;
            result->fruit_data.pose.orientation.x = transformed_pose.pose.orientation.x;
            result->fruit_data.pose.orientation.y = transformed_pose.pose.orientation.y;
            result->fruit_data.pose.orientation.z = transformed_pose.pose.orientation.z;
            result->fruit_data.pose.orientation.w = transformed_pose.pose.orientation.w;
            result->fruit_data.shape.dimensions.resize(2);
            result->fruit_data.shape.dimensions[0] = cameraFrameMsg.fruit_data.shape.dimensions[0];
            result->fruit_data.shape.dimensions[1] = cameraFrameMsg.fruit_data.shape.dimensions[1];

            if(hasPeduncleData){
                geometry_msgs::PoseStamped transformed_peduncle_pose = tf_buffer.transform(
                    peduncle_pose,
                    "world",
                    ros::Duration(3.0));
    
                result->peduncle_data.pose.position.x = transformed_peduncle_pose.pose.position.x;
                result->peduncle_data.pose.position.y = transformed_peduncle_pose.pose.position.y;
                result->peduncle_data.pose.position.z = transformed_peduncle_pose.pose.position.z;
                result->peduncle_data.pose.orientation.x = transformed_peduncle_pose.pose.orientation.x;
                result->peduncle_data.pose.orientation.y = transformed_peduncle_pose.pose.orientation.y;
                result->peduncle_data.pose.orientation.z = transformed_peduncle_pose.pose.orientation.z;
                result->peduncle_data.pose.orientation.w = transformed_peduncle_pose.pose.orientation.w;
                result->peduncle_data.shape.dimensions.resize(2);
                result->peduncle_data.shape.dimensions[0] = cameraFrameMsg.peduncle_data.shape.dimensions[0];
                result->peduncle_data.shape.dimensions[1] = cameraFrameMsg.peduncle_data.shape.dimensions[1];
            }
            // ROS_INFO("Transformed pose: x=%f, y=%f, z=%f",
            //          transformed_pose.pose.position.x,
            //          transformed_pose.pose.position.y,
            //  transformed_pose.pose.position.z);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failed: %s", ex.what());
        }
    }

    void visualizePepper(vader_msgs::Pepper pepper_estimate) {
        
        //Visualize as a visual tools marker
        std_msgs::ColorRGBA pepper_color;
        // green
        pepper_color.r = 0.0f;
        pepper_color.g = 1.0f;
        pepper_color.b = 0.0f;
        pepper_color.a = 0.8f; // semi-transparent
        ROS_INFO_STREAM("Visualizing pepper at: x=" << pepper_estimate.fruit_data.pose.position.x
                            << ", y=" << pepper_estimate.fruit_data.pose.position.y
                            << ", z=" << pepper_estimate.fruit_data.pose.position.z
                            << ", quat=(" << pepper_estimate.fruit_data.pose.orientation.x << ", "
                            << pepper_estimate.fruit_data.pose.orientation.y << ", "
                            << pepper_estimate.fruit_data.pose.orientation.z << ", "
                            << pepper_estimate.fruit_data.pose.orientation.w << ")");
        visual_tools->publishCylinder(pepper_estimate.fruit_data.pose, pepper_color, pepper_estimate.fruit_data.shape.dimensions[0], pepper_estimate.fruit_data.shape.dimensions[1]*2);
        if(pepper_estimate.peduncle_data.pose.position.x == 0 &&
           pepper_estimate.peduncle_data.pose.position.y == 0 &&
           pepper_estimate.peduncle_data.pose.position.z == 0){
            visual_tools->trigger();
            return;
        }
        std_msgs::ColorRGBA peduncle_color;
        // brown
        peduncle_color.r = 0.6f;
        peduncle_color.g = 0.3f;
        peduncle_color.b = 0.0f;
        peduncle_color.a = 0.8f; // semi-transparent
        visual_tools->publishCylinder(pepper_estimate.peduncle_data.pose, peduncle_color, pepper_estimate.peduncle_data.shape.dimensions[0], pepper_estimate.peduncle_data.shape.dimensions[1]*2);
        visual_tools->trigger();
    }

    void _sendGripperCommand(int open_pct)
    {
        vader_msgs::GripperCommand gripperCommand;
        gripperCommand.open_pct = open_pct;
        gripperCommandPub.publish(gripperCommand);
        ROS_INFO_STREAM("Gripper command: open_pct=" << open_pct);
    }

    void _sendCutterCommand(int open_pct)
    {
        vader_msgs::CutterCommand cutterCommand;
        cutterCommand.open_pct = open_pct;
        cutterCommandPub.publish(cutterCommand);
        ROS_INFO_STREAM("Cutter command: open_pct=" << open_pct);
    }

    void _logWithState(const std::string &message)
    {
        ROS_INFO_STREAM("[" << currentState << "]: " << message);
    }

    void _publishResultStatus(int result_code, const std::string &reason)
    {
        vader_msgs::HarvestResult resultMsg;
        resultMsg.result = result_code;
        resultMsg.reason = reason;
        resultStatusPub.publish(resultMsg);
        ros::Duration(1.0).sleep();
        resultStatusPub.publish(resultMsg);
        ROS_WARN_STREAM("Published harvest result: " << result_code << ", reason: " << reason);
    }

    bool _pepperReachable(const vader_msgs::Pepper &pepper)
    {
        if (SKIP_WORKSPACE_BOUNDS_CHECK){
            return true;
        }
        double x_min = 0.82;
        double x_max = 1.05;
        double y_min = 0;
        double y_max = 0.5;
        double z_min = 0.2;
        double z_max = 0.6;

        bool result = true;

        if (pepper.fruit_data.pose.position.x < x_min || pepper.fruit_data.pose.position.x > x_max)
        {
            // ROS_WARN_STREAM("Pepper x position out of reach: " << pepper.fruit_data.pose.position.x);
            result = false;
        }
        if (pepper.fruit_data.pose.position.y < y_min || pepper.fruit_data.pose.position.y > y_max)
        {
            // ROS_WARN_STREAM("Pepper y position out of reach: " << pepper.fruit_data.pose.position.y);
            result = false;
        }
        if (pepper.fruit_data.pose.position.z < z_min || pepper.fruit_data.pose.position.z > z_max)
        {
            // ROS_WARN_STREAM("Pepper z position out of reach: " << pepper.fruit_data.pose.position.z);
            result = false;
        }
        return result;
    }

    void _prioritizeCoarseEstimatePepper(std::vector<vader_msgs::Pepper> &peppers)
    {
        if (peppers.size() == 0)
        {
            return;
        }

        // Simple prioritization: choose the pepper closest to the center of the reachable workspace
        double x_center = 0.875; // (0.7 + 1.05) / 2
        double y_center = 0.25;  // (0 + 0.5) / 2
        double z_center = 0.45;  // (0.3 + 0.6) / 2

        double min_distance = std::numeric_limits<double>::max();
        int best_index = -1;

        for (size_t i = 0; i < peppers.size(); i++)
        {
            if(!_pepperReachable(peppers[i]))
            {
                continue;
            }
            double dx = peppers[i].fruit_data.pose.position.x - x_center;
            double dy = peppers[i].fruit_data.pose.position.y - y_center;
            double dz = peppers[i].fruit_data.pose.position.z - z_center;
            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < min_distance)
            {
                min_distance = distance;
                best_index = i;
            }
        }

        if (best_index != -1)
        {
            coarseEstimate = new vader_msgs::Pepper(peppers[best_index]);
            coarseEstimate->fruit_data.pose.orientation.x = 0;
            coarseEstimate->fruit_data.pose.orientation.y = 0;
            coarseEstimate->fruit_data.pose.orientation.z = 0;
            coarseEstimate->fruit_data.pose.orientation.w = 1;
            coarseEstimate->peduncle_data.pose.orientation.x = 0;
            coarseEstimate->peduncle_data.pose.orientation.y = 0;
            coarseEstimate->peduncle_data.pose.orientation.z = 0;
            coarseEstimate->peduncle_data.pose.orientation.w = 1;
            coarseEstimate->peduncle_data.pose.position.x = coarseEstimate->fruit_data.pose.position.x;
            coarseEstimate->peduncle_data.pose.position.y = coarseEstimate->fruit_data.pose.position.y;
            coarseEstimate->peduncle_data.pose.position.z = coarseEstimate->fruit_data.pose.position.z + 0.05; // approximate peduncle position
            // ROS_WARN_STREAM("Selected pepper ("
            //     << coarseEstimate->fruit_data.pose.position.x << ","
            //     << coarseEstimate->fruit_data.pose.position.y << "," 
            //     << coarseEstimate->fruit_data.pose.position.z << "," 
            //     << ") as the best coarse estimate.");
        }
    }

public:
    VADERStateMachine() : currentState(State::HomeGripper)
    {
        coarseEstimate = nullptr;
        coarseEstimateSub = n.subscribe("/gripper_coarse_pepper_array", 10, coarseEstimateCallback);
        fineEstimate = nullptr;
        fineEstimateSub = n.subscribe("/gripper_fine_pepper_array", 10, fineEstimateCallback);

        planClient = n.serviceClient<vader_msgs::PlanningRequest>("vader_planning_service");

        gripperCommandPub = n.advertise<vader_msgs::GripperCommand>("/gripper_command", 10);
        cutterCommandPub = n.advertise<vader_msgs::CutterCommand>("/cutter_command", 10);

        resultStatusPub = n.advertise<vader_msgs::HarvestResult>("/harvest_status", 10);
    }

    // void fakePepperPoseEstimate(const geometry_msgs::Pose &pose)
    // {
    //     vader_msgs::Pepper::Ptr msg(new vader_msgs::Pepper());
    //     msg->fruit_data.pose = pose;
    //     msg->header.frame_id = "world";
    //     msg->peduncle_data.pose = pose;
    //     msg->peduncle_data.pose.position.z += 0.1;
    //     msg->fruit_data.shape.dimensions.resize(2);
    //     msg->fruit_data.shape.dimensions[0] = 0.08;
    //     msg->fruit_data.shape.dimensions[1] = 0.035;
    //     msg->peduncle_data.shape.dimensions.resize(2);
    //     msg->peduncle_data.shape.dimensions[0] = 0.02;
    //     msg->peduncle_data.shape.dimensions[1] = 0.002;
    //     setCoarsePoseEstimate(msg);
    //     setFinePoseEstimate(msg);
    // }

    void setCoarsePoseEstimate(const vader_msgs::PepperArray::ConstPtr &msg)
    {
        if (!allowCoarseEstimateUpdate)
        {
            return;
        }
        coarseEstimates.clear();
        visual_tools->deleteAllMarkers();
        // ROS_INFO_STREAM("Received " << msg->peppers.size() << " coarse pepper estimates.");
        for (const vader_msgs::Pepper pepper_msg : msg->peppers)
        {
            vader_msgs::Pepper transformed_pepper;
            _transformFromCameraFrameIntoRobotFrame(pepper_msg, &transformed_pepper);
            transformed_pepper.fruit_data.pose.orientation.x = 0;
            transformed_pepper.fruit_data.pose.orientation.y = 0;
            transformed_pepper.fruit_data.pose.orientation.z = 0;
            transformed_pepper.fruit_data.pose.orientation.w = 1;
            coarseEstimates.push_back(transformed_pepper);
            
        }
        _prioritizeCoarseEstimatePepper(coarseEstimates);

        // if(coarseEstimate){
        // _transformFromCameraFrameIntoRobotFrame(msg, fineEstimate);
        // _logWithState("Coarse estimate received");
        // _logWithState("Raw message pose: x=" + std::to_string(msg->fruit_data.pose.position.x) +
        //               ", y=" + std::to_string(msg->fruit_data.pose.position.y) +
        //               ", z=" + std::to_string(msg->fruit_data.pose.position.z) + 
        //               ", quat=(" + std::to_string(msg->fruit_data.pose.orientation.x) + ", " +
        //               std::to_string(msg->fruit_data.pose.orientation.y) + ", " +
        //               std::to_string(msg->fruit_data.pose.orientation.z) + ", " +
        //               std::to_string(msg->fruit_data.pose.orientation.w) + ")");
            // _logWithState("Transformed pose: x=" + std::to_string(coarseEstimate->fruit_data.pose.position.x) +
            //             ", y=" + std::to_string(coarseEstimate->fruit_data.pose.position.y) +
            //             ", z=" + std::to_string(coarseEstimate->fruit_data.pose.position.z) + 
            //             ", quat=(" + std::to_string(coarseEstimate->fruit_data.pose.orientation.x) + ", " +
            //             std::to_string(coarseEstimate->fruit_data.pose.orientation.y) + ", " +
            //             std::to_string(coarseEstimate->fruit_data.pose.orientation.z) + ", " +
            //             std::to_string(coarseEstimate->fruit_data.pose.orientation.w) + ")");
        // }
    }

    void setFinePoseEstimate(const vader_msgs::PepperArray::ConstPtr &msg)
    {
        if (!allowFineEstimateUpdate)
        {
            return;
        }
        fineEstimates.clear();
        // ROS_INFO_STREAM("Received " << msg->peppers.size() << " fine pepper estimates.");
        for (const vader_msgs::Pepper pepper_msg : msg->peppers)
        {
            if(pepper_msg.peduncle_data.pose.position.x == 0 &&
               pepper_msg.peduncle_data.pose.position.y == 0 &&
               pepper_msg.peduncle_data.pose.position.z == 0){
                continue;
            }
            vader_msgs::Pepper transformed_pepper;
            _transformFromCameraFrameIntoRobotFrame(pepper_msg, &transformed_pepper);
            fineEstimates.push_back(transformed_pepper);
            // visualizePepper(transformed_pepper);
        }
        // ROS_INFO_STREAM("Transformed fine estimates count: " << fineEstimates.size());

        if (!fineEstimates.empty() && coarseEstimate != nullptr)
        {
            // Take the first valid fine estimate
            for (const auto& pepper : fineEstimates)
            {
                double distToCoarseEstimate = sqrt(
                    pow(pepper.fruit_data.pose.position.x - coarseEstimate->fruit_data.pose.position.x, 2) +
                    pow(pepper.fruit_data.pose.position.y - coarseEstimate->fruit_data.pose.position.y, 2) +
                    pow(pepper.fruit_data.pose.position.z - coarseEstimate->fruit_data.pose.position.z, 2));
                if (_pepperReachable(pepper) && distToCoarseEstimate < 0.1)
                {
                    fineEstimate = new vader_msgs::Pepper(pepper);
                
                    // compute rotation angle from quaternion relative to identity
                    const auto &q = fineEstimate->fruit_data.pose.orientation;
                    double qw = q.w;
                    double angle_rad = 2.0 * acos(fabs(qw));
                    double angle_deg = angle_rad * 180.0 / M_PI;
                    // ROS_INFO_STREAM("Fine estimate rotation magnitude: " << angle_deg << " degrees");
                    if (angle_deg > REJECT_FINE_POSE_BEYOND_ANGLE_DEGREES && angle_deg < (180.0 - REJECT_FINE_POSE_BEYOND_ANGLE_DEGREES)) {
                        fineEstimate->fruit_data.pose.orientation.x = 0.0;
                        fineEstimate->fruit_data.pose.orientation.y = 0.0;
                        fineEstimate->fruit_data.pose.orientation.z = 0.0;
                        fineEstimate->fruit_data.pose.orientation.w = 1.0;
                        
                        fineEstimate->peduncle_data.pose.position.x = fineEstimate->fruit_data.pose.position.x;
                        fineEstimate->peduncle_data.pose.position.y = fineEstimate->fruit_data.pose.position.y;
                        fineEstimate->peduncle_data.pose.position.z = fineEstimate->fruit_data.pose.position.z + 0.035;
                        fineEstimate->peduncle_data.pose.orientation.x = 0.0;
                        fineEstimate->peduncle_data.pose.orientation.y = 0.0;
                        fineEstimate->peduncle_data.pose.orientation.z = 0.0;
                        fineEstimate->peduncle_data.pose.orientation.w = 1.0;
                        ROS_WARN_STREAM("Rotation > " << REJECT_FINE_POSE_BEYOND_ANGLE_DEGREES << " deg; resetting fine estimate fruit orientation to identity");
                    }
                
                    // ROS_INFO_STREAM("Selected fine estimate: x=" << fineEstimate->fruit_data.pose.position.x
                    //             << ", y=" << fineEstimate->fruit_data.pose.position.y
                    //             << ", z=" << fineEstimate->fruit_data.pose.position.z
                    //             << ", quat=(" << fineEstimate->fruit_data.pose.orientation.x << ", "
                    //             << fineEstimate->fruit_data.pose.orientation.y << ", "
                    //             << fineEstimate->fruit_data.pose.orientation.z << ", "
                    //             << fineEstimate->fruit_data.pose.orientation.w << ")");
                    break;
                }
            }
        }
    }

    vader_msgs::Pepper setFinePoseEstimateWithCoarsePoseEst(const vader_msgs::Pepper &coarseEstimate){
        vader_msgs::Pepper fakeFineEstimate = (coarseEstimate);
        fakeFineEstimate.header.frame_id = "world";
        fakeFineEstimate.fruit_data.pose.orientation.x = 0;
        fakeFineEstimate.fruit_data.pose.orientation.y = 0;
        fakeFineEstimate.fruit_data.pose.orientation.z = 0;
        fakeFineEstimate.fruit_data.pose.orientation.w = 1;
        fakeFineEstimate.peduncle_data.pose.orientation.x = 0;
        fakeFineEstimate.peduncle_data.pose.orientation.y = 0;
        fakeFineEstimate.peduncle_data.pose.orientation.z = 0;
        fakeFineEstimate.peduncle_data.pose.orientation.w = 1;
        fakeFineEstimate.peduncle_data.pose.position.x = fakeFineEstimate.fruit_data.pose.position.x;
        fakeFineEstimate.peduncle_data.pose.position.y = fakeFineEstimate.fruit_data.pose.position.y;
        fakeFineEstimate.peduncle_data.pose.position.z = fakeFineEstimate.fruit_data.pose.position.z + 0.05; // approximate peduncle position
        return fakeFineEstimate;
    }

    void execute()
    {
        int num_peppers_harvested = 0;
        int max_peppers_to_harvest = 99;
        std::vector<double> harvest_times_sec;

        //Start time of each harvest
        ros::Time harvest_start_time;

        ros::Time printCoarsePoseWaitingMsgTime = ros::Time(0);

        ros::Rate loop_rate(20);
        while (ros::ok())
        {
            switch (currentState)
            {
                case State::HomeGripper:{
                    harvest_start_time = ros::Time::now();
                    allowCoarseEstimateUpdate = true;
                    allowFineEstimateUpdate = true;
                    _logWithState("Homing gripper...");
                    _sendGripperCommand(100);
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::HOME_GRIPPER;
                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Gripper homed successfully.");
                            currentState = State::HomeCutter;
                        }
                        else
                        {
                            _logWithState("Gripper homing failed.");
                            ROS_ERROR_NAMED("vader_hri", "Trying homing again...");
                            currentState = State::HomeGripper;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to home gripper.");
                        _logWithState("Failed to call planning service for gripper homing.");
                        ROS_ERROR_NAMED("vader_hri", "Trying homing again...");
                        currentState = State::HomeGripper;
                    }
                    break;
                }
                case State::HomeCutter:{
                    _logWithState("Homing cutter...");
                    _sendCutterCommand(100);
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::HOME_CUTTER;
                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Cutter homed successfully.");
                            currentState = State::WaitForCoarseEstimate;
                        }
                        else
                        {
                            _logWithState("Cutter homing failed.");
                            ROS_ERROR_NAMED("vader_hri", "Trying homing again...");
                            currentState = State::HomeCutter;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to home cutter.");
                        _logWithState("Failed to call planning service for cutter homing.");
                        ROS_ERROR_NAMED("vader_hri", "Trying homing again...");
                        currentState = State::HomeCutter;
                    }
                    break;
                }
                case State::WaitForCoarseEstimate:{
                    if (printCoarsePoseWaitingMsgTime.isZero()){
                        //Start tracking print timer
                        printCoarsePoseWaitingMsgTime = ros::Time::now();
                    }
                    if (coarseEstimate != nullptr)
                    {
                        _logWithState("Coarse estimate received, switching states");
                        _logWithState("Transformed pose: x=" + std::to_string(coarseEstimate->fruit_data.pose.position.x) +
                                      ", y=" + std::to_string(coarseEstimate->fruit_data.pose.position.y) +
                                      ", z=" + std::to_string(coarseEstimate->fruit_data.pose.position.z) + 
                                      ", quat=(" + std::to_string(coarseEstimate->fruit_data.pose.orientation.x) + ", " +
                                      std::to_string(coarseEstimate->fruit_data.pose.orientation.y) + ", " +
                                      std::to_string(coarseEstimate->fruit_data.pose.orientation.z) + ", " +
                                      std::to_string(coarseEstimate->fruit_data.pose.orientation.w) + ")");
                        printCoarsePoseWaitingMsgTime = ros::Time(0);
                        visual_tools->deleteAllMarkers();
                        currentState = State::ParallelMovePregrasp;
                    }
                    else
                    {
                        if((ros::Time::now() - printCoarsePoseWaitingMsgTime).toSec() > 3.0){
                            printCoarsePoseWaitingMsgTime = ros::Time::now();
                            _logWithState("Still waiting for coarse estimate");
                            if(!coarseEstimates.empty()){
                                ROS_INFO_STREAM("NOTE: " << coarseEstimates.size() << " coarse estimates found but outside of workspace bounds.");
                            }
                        }
                    }
                    break;
                }
                case State::ParallelMovePregrasp:{
                    allowCoarseEstimateUpdate = false;
                    _logWithState("Planning parallel move to pregrasp...");
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_PREGRASP;
                    srv.request.pepper = *coarseEstimate;

                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Parallel move to pregrasp successful.");
                            currentState = State::WaitForFineEstimate;
                            waitForFinePoseStartTime = ros::Time::now();
                        }
                        else
                        {
                            _logWithState("Parallel move to pregrasp failed.");
                            ROS_ERROR_NAMED("vader_hri", "Falling back to Home Poses and trying again...");
                            currentState = State::HomeGripper;
                        }
                    }
                    else
                    {
                        //Call to planner itself failed (not that planner failed). Something with ROS is wrong. We should never see this
                        _publishResultStatus(srv.response.success, "Failed to move to pregrasp.");
                        _logWithState("Failed to call planning service for parallel move to pregrasp.");
                        ROS_ERROR_NAMED("vader_hri", "Falling back to Home Poses and trying again...");
                        currentState = State::HomeGripper;
                    }
                    break;
                }
                case State::WaitForFineEstimate:{
                    if (fineEstimate != nullptr)
                    {
                        _logWithState("Fine estimate received, switching states");
                        _logWithState("Transformed FINE pose: x=" + std::to_string(fineEstimate->fruit_data.pose.position.x) +
                                      ", y=" + std::to_string(fineEstimate->fruit_data.pose.position.y) +
                                      ", z=" + std::to_string(fineEstimate->fruit_data.pose.position.z) + 
                                      ", quat=(" + std::to_string(fineEstimate->fruit_data.pose.orientation.x) + ", " +
                                      std::to_string(fineEstimate->fruit_data.pose.orientation.y) + ", " +
                                      std::to_string(fineEstimate->fruit_data.pose.orientation.z) + ", " +
                                      std::to_string(fineEstimate->fruit_data.pose.orientation.w) + ")");

                        allowFineEstimateUpdate = false;
                        waitForFinePoseStartTime = ros::Time(0);
                        currentState = State::GripperGrasp;
                        visual_tools->deleteAllMarkers();
                        visualizePepper(*fineEstimate);
                    }
                    else
                    {
                        if(!waitForFinePoseStartTime.isZero()){
                            ros::Duration wait_duration = ros::Time::now() - waitForFinePoseStartTime;
                            if(wait_duration.toSec() > 3.0 && allowFineEstimateUpdate){
                                ROS_WARN("Timeout waiting for fine pose estimate. Proceeding with coarse estimate.");
                                vader_msgs::Pepper fakeFineEstimate = setFinePoseEstimateWithCoarsePoseEst(*coarseEstimate);
                                fineEstimate = new vader_msgs::Pepper(fakeFineEstimate);
                                allowFineEstimateUpdate = false;
                                
                                // *fineEstimate = setFinePoseEstimateWithCoarsePoseEst(*coarseEstimate);
                            }
                        }
                    }
                    break;
                }
                case State::GripperGrasp:{
                    _logWithState("Planning gripper grasp...");
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::GRIPPER_GRASP;
                    srv.request.pepper = *fineEstimate;
                    ROS_INFO_STREAM("Fine estimate for grasp: x=" << fineEstimate->fruit_data.pose.position.x
                                    << ", y=" << fineEstimate->fruit_data.pose.position.y
                                    << ", z=" << fineEstimate->fruit_data.pose.position.z
                                    << ", quat=(" << fineEstimate->fruit_data.pose.orientation.x << ", "
                                    << fineEstimate->fruit_data.pose.orientation.y << ", "
                                    << fineEstimate->fruit_data.pose.orientation.z << ", "
                                    << fineEstimate->fruit_data.pose.orientation.w << ")");

                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Gripper grasp successful.");
                            currentState = State::CutterGrasp;
                        }
                        else
                        {
                            _logWithState("Gripper grasp failed.");
                            ROS_ERROR_NAMED("vader_hri", "Falling back to Home Poses and trying again...");
                            currentState = State::HomeGripper;
                        }
                    }
                    else
                    {
                        //Call to planner itself failed (not that planner failed). Something with ROS is wrong. We should never see this
                        _publishResultStatus(srv.response.success, "Failed to gripper grasp.");
                        ROS_ERROR_NAMED("vader_hri", "Falling back to Home Poses and trying again...");
                        currentState = State::HomeGripper;
                    }
                    break;
                }
                case State::CutterGrasp:{
                    _logWithState("Planning cutter grasp...");
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::CUTTER_GRASP;
                    srv.request.pepper = *fineEstimate;

                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Cutter grasp successful.");
                            currentState = State::GripperEndEffector;  // First do the cutting action
                        }
                        else
                        {
                            _logWithState("Cutter grasp failed.");
                            ROS_ERROR_NAMED("vader_hri", "Falling back to Home Poses and trying again...");
                            currentState = State::HomeGripper;
                        }
                    }
                    else
                    {
                        //Call to planner itself failed (not that planner failed). Something with ROS is wrong. We should never see this
                        _publishResultStatus(srv.response.success, "Failed to cutter grasp.");
                        ROS_ERROR_NAMED("vader_hri", "Falling back to Home Poses and trying again...");
                        currentState = State::HomeGripper;
                    }
                    break;
                }
                case State::GripperEndEffector:
                {
                    _logWithState("Grasping fruit");
                    _sendGripperCommand(20);
                    ros::Duration(3.0).sleep();
                    // _sendGripperCommand(100);
                    // ros::Duration(1.0).sleep();
                    currentState = State::CutterEndEffector;
                    break;
                }
                case State::CutterEndEffector:
                {
                    _logWithState("Cutting peduncle");
                    _sendCutterCommand(0);
                    ros::Duration(2.0).sleep();
                    _sendCutterCommand(100);
                    ros::Duration(2.0).sleep();
                    _sendCutterCommand(0);
                    ros::Duration(2.0).sleep();
                    _sendCutterCommand(100);
                    ros::Duration(2.0).sleep();
                    // _sendGripperCommand(100);
                    // ros::Duration(1.0).sleep();
                    // currentState = State::Done;
                    currentState = State::ParallelMoveStorage;
                    break;
                }
                case State::ParallelMoveStorage:{
                    _logWithState("Planning parallel move to storage...");
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_STORAGE;

                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Parallel move to storage successful.");
                            currentState = State::HomeGripper2;
                        }
                        else
                        {
                            _logWithState("Parallel move to storage failed.");
                            ROS_ERROR_NAMED("vader_hri", "Moving to storage failed!! Continuing to move home but pepper may have been lost.");
                            currentState = State::HomeGripper2;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to move to storage.");
                        _logWithState("Failed to call planning service for parallel move to storage.");
                        currentState = State::Error;
                    }
                    break;
                }
                case State::HomeGripper2:{
                    _logWithState("Homing gripper after storage...");
                    _sendGripperCommand(100);
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::HOME_GRIPPER;
                    if (planClient.call(srv))
                    {
                        if (srv.response.success)
                        {
                            _logWithState("Gripper homed successfully.");
                            currentState = State::Done;
                        }
                        else
                        {
                            _logWithState("Gripper homing failed.");
                            currentState = State::Error;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to home gripper.");
                        _logWithState("Failed to call planning service for gripper homing.");
                        currentState = State::Error;
                    }
                    break;
                }
                
                case State::Done:
                {
                    _publishResultStatus(vader_msgs::HarvestResult::RESULT_SUCCESS, "Success!");
                    num_peppers_harvested += 1;
                    coarseEstimate = nullptr;
                    fineEstimate = nullptr;
                    ros::Duration harvest_time = ros::Time::now() - harvest_start_time;
                    harvest_times_sec.push_back(harvest_time.toSec());
                    ROS_INFO_STREAM("Harvest time for pepper " << num_peppers_harvested << ": " << harvest_time.toSec() << " seconds");
                    if (num_peppers_harvested >= max_peppers_to_harvest)
                    {
                        currentState = State::End;
                        _logWithState("Harvested enough peppers. End of state machine execution.");
                        double total_time = 0.0;
                        for(int i = 0; i < harvest_times_sec.size(); i++) {
                            ROS_INFO_STREAM("Harvest time for pepper " << (i+1) << ": " << harvest_times_sec[i] << " seconds");
                            total_time += harvest_times_sec[i];
                        }
                        ROS_INFO_STREAM("Average harvest time: " << (total_time / harvest_times_sec.size()) << " seconds");
                    }
                    else
                    {
                        currentState = State::HomeGripper;
                    }
                    _logWithState("Done");
                    break;
                }
                case State::Error:
                {
                    break;
                }
                case State::End:
                {
                    break;
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

VADERStateMachine *sm = nullptr;

static void coarseEstimateCallback(const vader_msgs::PepperArray::ConstPtr &msg)
{
    // ROS_INFO("Coarse estimate callback triggered");
    if (sm != nullptr)
    {
        sm->setCoarsePoseEstimate(msg);
    }
    else
    {
        ROS_ERROR("State machine is not initialized");
    }
}

static void fineEstimateCallback(const vader_msgs::PepperArray::ConstPtr &msg)
{
    if (sm != nullptr)
    {
        sm->setFinePoseEstimate(msg);
    }
    else
    {
        ROS_ERROR("State machine is not initialized");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vader_hri");
    VADERStateMachine statemachine = VADERStateMachine();
    sm = &statemachine;
    ros::Duration(20.0).sleep(); //wait for stuff
    statemachine.execute();
    return 0;
}