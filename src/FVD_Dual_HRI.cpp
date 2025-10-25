#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"
// #include "vader_msgs/BimanualPlanRequest.h"
// #include "vader_msgs/BimanualExecRequest.h"
// #include "vader_msgs/MoveToStorageRequest.h"
#include "vader_msgs/GripperCommand.h"
#include "vader_msgs/CutterCommand.h"

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
        // ------------Harvest stage-----------
        CutterGrasp,
        // -----------Finish and cleanup-----------
        ParallelMoveStorage,
        Done,
        Error
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

    /*
    Input: cameraFrameMsg containing the pose of the fruit in the camera frame from the coarse/fine pepper estimate
    Output: result containing the pose of the fruit in the world frame using TF2.
    */
    void _transformFromCameraFrameIntoRobotFrame(const vader_msgs::Pepper::ConstPtr &cameraFrameMsg, vader_msgs::Pepper *result)
    {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        geometry_msgs::PoseStamped fruit_pose;
        //TODO The below x/y swap/weirdness is a weird artifact left from previous PR4 work. Should be removed, but also need ot change sim logic everywhere
        //So that the coordinates are correct with x/y unswapped.
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

public:
    VADERStateMachine() : currentState(State::HomeGripper)
    {
        coarseEstimate = nullptr;
        coarseEstimateSub = n.subscribe("/fruit_coarse_pose", 10, coarseEstimateCallback);
        fineEstimate = nullptr;
        fineEstimateSub = n.subscribe("/fruit_fine_pose", 10, fineEstimateCallback);

        planClient = n.serviceClient<vader_msgs::PlanningRequest>("vader_planning_service");

        gripperCommandPub = n.advertise<vader_msgs::GripperCommand>("/gripper_command", 10);
        cutterCommandPub = n.advertise<vader_msgs::CutterCommand>("/cutter_command", 10);
    }

    void setCoarsePoseEstimate(const vader_msgs::Pepper::ConstPtr &msg)
    {
        coarseEstimate = new vader_msgs::Pepper();
        coarseEstimate->header = msg->header;
        _transformFromCameraFrameIntoRobotFrame(msg, coarseEstimate);
        // _logWithState("Coarse estimate received");
    }

    void setFinePoseEstimate(const vader_msgs::Pepper::ConstPtr &msg)
    {
        fineEstimate = new vader_msgs::Pepper();
        fineEstimate->header = msg->header;
        _transformFromCameraFrameIntoRobotFrame(msg, fineEstimate);
        // _logWithState("Fine estimate received");
    }

    void execute()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            switch (currentState)
            {
                case State::HomeGripper:{
                    _logWithState("Homing gripper...");
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
                        _logWithState("Failed to call planning service for gripper homing.");
                        currentState = State::Error;
                    }
                    break;
                }
                case State::HomeCutter:{
                    _logWithState("Homing cutter...");
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
                            currentState = State::Done;
                        }
                        else
                        {
                            _logWithState("Parallel move to pregrasp failed.");
                            currentState = State::Error;
                        }
                    }
                    else
                    {
                        _logWithState("Failed to call planning service for parallel move to pregrasp.");
                        currentState = State::Error;
                    }
                    break;
                }
                case State::WaitForFineEstimate:{

                }
                case State::GripperGrasp:{

                }
                case State::CutterGrasp:{

                }
                case State::ParallelMoveStorage:{

                }
                case State::Done:
                {
                    // ROS_INFO("Done");
                    _logWithState("Done");
                    break;
                }
                case State::Error:
                {
                    // ROS_INFO("Error");
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
    ros::Duration(10.0).sleep(); //wait for stuff
    statemachine.execute();
    return 0;
}