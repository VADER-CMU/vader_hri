#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"
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

static void coarseEstimateCallback(const vader_msgs::Pepper::ConstPtr &msg);
static void fineEstimateCallback(const vader_msgs::Pepper::ConstPtr &msg);

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
    vader_msgs::Pepper *coarseEstimate;
    vader_msgs::Pepper *fineEstimate;

    // Plan/Exec clients connecting to planner
    ros::ServiceClient planClient;

    // End effector talking to dynamixel node
    ros::Publisher gripperCommandPub, cutterCommandPub;

    ros::Publisher resultStatusPub; //publishes to /harvest_result

    /*
    Input: cameraFrameMsg containing the pose of the fruit in the camera frame from the coarse/fine pepper estimate
    Output: result containing the pose of the fruit in the world frame using TF2.
    */
    void _transformFromCameraFrameIntoRobotFrame(const vader_msgs::Pepper::ConstPtr &cameraFrameMsg, vader_msgs::Pepper *result)
    {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        geometry_msgs::PoseStamped fruit_pose;
        fruit_pose.pose.position.x = cameraFrameMsg->fruit_data.pose.position.x;
        fruit_pose.pose.position.y = cameraFrameMsg->fruit_data.pose.position.y;
        fruit_pose.pose.position.z = cameraFrameMsg->fruit_data.pose.position.z;
        fruit_pose.pose.orientation.x = cameraFrameMsg->fruit_data.pose.orientation.x;
        fruit_pose.pose.orientation.y = cameraFrameMsg->fruit_data.pose.orientation.y;
        fruit_pose.pose.orientation.z = cameraFrameMsg->fruit_data.pose.orientation.z;
        fruit_pose.pose.orientation.w = cameraFrameMsg->fruit_data.pose.orientation.w;

        geometry_msgs::PoseStamped peduncle_pose;
        peduncle_pose.pose.position.x = cameraFrameMsg->peduncle_data.pose.position.x;
        peduncle_pose.pose.position.y = cameraFrameMsg->peduncle_data.pose.position.y;
        peduncle_pose.pose.position.z = cameraFrameMsg->peduncle_data.pose.position.z;
        peduncle_pose.pose.orientation.x = cameraFrameMsg->peduncle_data.pose.orientation.x;
        peduncle_pose.pose.orientation.y = cameraFrameMsg->peduncle_data.pose.orientation.y;
        peduncle_pose.pose.orientation.z = cameraFrameMsg->peduncle_data.pose.orientation.z;
        peduncle_pose.pose.orientation.w = cameraFrameMsg->peduncle_data.pose.orientation.w;
        fruit_pose.header.frame_id = cameraFrameMsg->header.frame_id;
        peduncle_pose.header.frame_id = cameraFrameMsg->header.frame_id;

        try
        {
            geometry_msgs::PoseStamped transformed_pose = tf_buffer.transform(
                fruit_pose,
                "world",
                ros::Duration(3.0));
            geometry_msgs::PoseStamped transformed_peduncle_pose = tf_buffer.transform(
                peduncle_pose,
                "world",
                ros::Duration(3.0));
            // ROS_INFO("Transformed pose: x=%f, y=%f, z=%f",
            //          transformed_pose.pose.position.x,
            //          transformed_pose.pose.position.y,
            //  transformed_pose.pose.position.z);
            result->fruit_data.pose.position.x = transformed_pose.pose.position.x;
            result->fruit_data.pose.position.y = transformed_pose.pose.position.y;
            result->fruit_data.pose.position.z = transformed_pose.pose.position.z;
            result->fruit_data.pose.orientation.x = transformed_pose.pose.orientation.x;
            result->fruit_data.pose.orientation.y = transformed_pose.pose.orientation.y;
            result->fruit_data.pose.orientation.z = transformed_pose.pose.orientation.z;
            result->fruit_data.pose.orientation.w = transformed_pose.pose.orientation.w;
            result->fruit_data.shape.dimensions.resize(2);
            result->fruit_data.shape.dimensions[0] = cameraFrameMsg->fruit_data.shape.dimensions[0];
            result->fruit_data.shape.dimensions[1] = cameraFrameMsg->fruit_data.shape.dimensions[1];

            result->peduncle_data.pose.position.x = transformed_peduncle_pose.pose.position.x;
            result->peduncle_data.pose.position.y = transformed_peduncle_pose.pose.position.y;
            result->peduncle_data.pose.position.z = transformed_peduncle_pose.pose.position.z;
            result->peduncle_data.pose.orientation.x = transformed_peduncle_pose.pose.orientation.x;
            result->peduncle_data.pose.orientation.y = transformed_peduncle_pose.pose.orientation.y;
            result->peduncle_data.pose.orientation.z = transformed_peduncle_pose.pose.orientation.z;
            result->peduncle_data.pose.orientation.w = transformed_peduncle_pose.pose.orientation.w;
            result->peduncle_data.shape.dimensions.resize(2);
            result->peduncle_data.shape.dimensions[0] = cameraFrameMsg->peduncle_data.shape.dimensions[0];
            result->peduncle_data.shape.dimensions[1] = cameraFrameMsg->peduncle_data.shape.dimensions[1];
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failed: %s", ex.what());
        }
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

public:
    VADERStateMachine() : currentState(State::HomeGripper)
    {
        coarseEstimate = nullptr;
        coarseEstimateSub = n.subscribe("/gripper_coarse_pose", 10, coarseEstimateCallback);
        fineEstimate = nullptr;
        fineEstimateSub = n.subscribe("/fruit_fine_pose", 10, fineEstimateCallback);

        planClient = n.serviceClient<vader_msgs::PlanningRequest>("vader_planning_service");

        gripperCommandPub = n.advertise<vader_msgs::GripperCommand>("/gripper_command", 10);
        cutterCommandPub = n.advertise<vader_msgs::CutterCommand>("/cutter_command", 10);

        resultStatusPub = n.advertise<vader_msgs::HarvestResult>("/harvest_status", 10);
    }

    void fakePepperPoseEstimate(const geometry_msgs::Pose &pose)
    {
        vader_msgs::Pepper::Ptr msg(new vader_msgs::Pepper());
        msg->fruit_data.pose = pose;
        msg->header.frame_id = "world";
        msg->peduncle_data.pose = pose;
        msg->peduncle_data.pose.position.z += 0.1;
        msg->fruit_data.shape.dimensions.resize(2);
        msg->fruit_data.shape.dimensions[0] = 0.08;
        msg->fruit_data.shape.dimensions[1] = 0.035;
        msg->peduncle_data.shape.dimensions.resize(2);
        msg->peduncle_data.shape.dimensions[0] = 0.02;
        msg->peduncle_data.shape.dimensions[1] = 0.002;
        setCoarsePoseEstimate(msg);
        setFinePoseEstimate(msg);
    }

    void setCoarsePoseEstimate(const vader_msgs::Pepper::ConstPtr &msg)
    {
        coarseEstimate = new vader_msgs::Pepper();
        coarseEstimate->header = msg->header;
        // fineEstimate = new vader_msgs::Pepper();
        // fineEstimate->header = msg->header;
        _transformFromCameraFrameIntoRobotFrame(msg, coarseEstimate);
        // _transformFromCameraFrameIntoRobotFrame(msg, fineEstimate);
        _logWithState("Coarse estimate received");
        _logWithState("Raw message pose: x=" + std::to_string(msg->fruit_data.pose.position.x) +
                      ", y=" + std::to_string(msg->fruit_data.pose.position.y) +
                      ", z=" + std::to_string(msg->fruit_data.pose.position.z) + 
                      ", quat=(" + std::to_string(msg->fruit_data.pose.orientation.x) + ", " +
                      std::to_string(msg->fruit_data.pose.orientation.y) + ", " +
                      std::to_string(msg->fruit_data.pose.orientation.z) + ", " +
                      std::to_string(msg->fruit_data.pose.orientation.w) + ")");
        _logWithState("Transformed pose: x=" + std::to_string(coarseEstimate->fruit_data.pose.position.x) +
                      ", y=" + std::to_string(coarseEstimate->fruit_data.pose.position.y) +
                      ", z=" + std::to_string(coarseEstimate->fruit_data.pose.position.z) + 
                      ", quat=(" + std::to_string(coarseEstimate->fruit_data.pose.orientation.x) + ", " +
                      std::to_string(coarseEstimate->fruit_data.pose.orientation.y) + ", " +
                      std::to_string(coarseEstimate->fruit_data.pose.orientation.z) + ", " +
                      std::to_string(coarseEstimate->fruit_data.pose.orientation.w) + ")");
    }

    void setFinePoseEstimate(const vader_msgs::Pepper::ConstPtr &msg)
    {
        fineEstimate = new vader_msgs::Pepper();
        fineEstimate->header = msg->header;
        _transformFromCameraFrameIntoRobotFrame(msg, fineEstimate);
        _logWithState("Fine estimate received");
    }

    void execute()
    {
        int num_peppers_harvested = 0;
        int max_peppers_to_harvest = 3;
        std::vector<double> harvest_times_sec;

        //Start time of each harvest
        ros::Time harvest_start_time;


        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            switch (currentState)
            {
                case State::HomeGripper:{
                    harvest_start_time = ros::Time::now();
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
                            currentState = State::Error;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to home cutter.");
                        _logWithState("Failed to call planning service for cutter homing.");
                        currentState = State::Error;
                    }
                    break;
                }
                case State::WaitForCoarseEstimate:{
                    if (coarseEstimate != nullptr)
                    {
                        _logWithState("Coarse estimate received, switching states");
                        currentState = State::ParallelMovePregrasp;
                    }
                    else
                    {
                        _logWithState("Waiting for coarse estimate");
                        _logWithState("Using fake estimate");
                        geometry_msgs::Pose fake_pose;
                        fake_pose.position.x = 1.0;
                        fake_pose.position.y = 0.2;
                        fake_pose.position.z = 0.3;
                        fake_pose.orientation.x = 0.0;
                        fake_pose.orientation.y = 0.0;
                        fake_pose.orientation.z = 0.0;
                        fake_pose.orientation.w = 1.0;
                        fakePepperPoseEstimate(fake_pose);
                    }
                    break;
                }
                case State::ParallelMovePregrasp:{
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
                        }
                        else
                        {
                            _logWithState("Parallel move to pregrasp failed.");
                            currentState = State::Error;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to move to pregrasp.");
                        _logWithState("Failed to call planning service for parallel move to pregrasp.");
                        currentState = State::Error;
                    }
                    break;
                }
                case State::WaitForFineEstimate:{
                    if (fineEstimate != nullptr)
                    {
                        _logWithState("Fine estimate received, switching states");
                        currentState = State::GripperGrasp;
                    }
                    else
                    {
                        _logWithState("Waiting for fine estimate");
                    }
                    break;
                }
                case State::GripperGrasp:{
                    _logWithState("Planning gripper grasp...");
                    vader_msgs::PlanningRequest srv;
                    srv.request.mode = vader_msgs::PlanningRequest::Request::GRIPPER_GRASP;
                    srv.request.pepper = *fineEstimate;

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
                            currentState = State::Error;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to gripper grasp.");
                        _logWithState("Failed to call planning service for gripper grasp.");
                        currentState = State::Error;
                    }
                    break;
                }
                case State::GripperEndEffector:
                {
                    _logWithState("Grasping fruit");
                    _sendGripperCommand(0);
                    ros::Duration(5.0).sleep();
                    // _sendGripperCommand(100);
                    // ros::Duration(1.0).sleep();
                    currentState = State::CutterGrasp;
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
                            currentState = State::ParallelMoveStorage;
                        }
                        else
                        {
                            _logWithState("Cutter grasp failed.");
                            currentState = State::Error;
                        }
                    }
                    else
                    {
                        _publishResultStatus(srv.response.success, "Failed to cutter grasp.");
                        _logWithState("Failed to call planning service for cutter grasp.");
                        currentState = State::Error;
                    }
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
                            currentState = State::Error;
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
                    // ROS_INFO("Done");
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

static void coarseEstimateCallback(const vader_msgs::Pepper::ConstPtr &msg)
{
    if (sm != nullptr)
    {
        sm->setCoarsePoseEstimate(msg);
    }
    else
    {
        ROS_ERROR("State machine is not initialized");
    }
}

static void fineEstimateCallback(const vader_msgs::Pepper::ConstPtr &msg)
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