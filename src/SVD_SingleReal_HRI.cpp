#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"
#include "vader_msgs/SingleArmPlanRequest.h"
#include "vader_msgs/SingleArmExecutionRequest.h"
#include "vader_msgs/MoveToStorageRequest.h"
#include "vader_msgs/GoHomeRequest.h"
#include "vader_msgs/GripperCommand.h"
#include "vader_msgs/CutterCommand.h"

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
        WaitForCoarseEstimate,
        PlanGripperToPregrasp,
        MoveGripperToPregrasp,
        // ----------Fine-Grasp stage-----------
        WaitForFineEstimate,
        PlanGripperToGrasp,
        MoveGripperToGrasp,
        GripperGrasp,
        // -----------Finish and cleanup-----------
        PlanAndMoveToBin,
        GripperRelease,
        Done,
        Home,
        Error
    };

    State currentState;
    ros::NodeHandle n;

    // Perception subscriptions and data
    ros::Subscriber coarseEstimateSub, fineEstimateSub;
    vader_msgs::Pepper *coarseEstimate;
    vader_msgs::Pepper *fineEstimate;
    geometry_msgs::Pose *storageBinLocation; // only the fruit pose is used to designate storage bin location.

    // geometry_msgs::Pose* homeLocation;

    // Plan/Exec clients connecting to planner
    ros::ServiceClient planClient, execClient;
    ros::ServiceClient moveToStorageClient;
    ros::ServiceClient goHomeClient;

    // End effector talking to dynamixel node
    ros::Publisher gripperCommandPub, cutterCommandPub;

    /*
    Input: cameraFrameMsg containing the pose of the fruit in the camera frame from the coarse/fine pepper estimate
    Output: result containing the pose of the fruit in the world frame using TF2.
    */
    void _transformFromCameraFrameIntoRobotFrame(const vader_msgs::Pepper::ConstPtr &cameraFrameMsg, vader_msgs::Pepper *result, bool isCoarse)
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
                "link_base",
                ros::Duration(3.0));
            geometry_msgs::PoseStamped transformed_peduncle_pose = tf_buffer.transform(
                peduncle_pose,
                "link_base",
                ros::Duration(3.0));
            // ROS_INFO("Transformed pose: x=%f, y=%f, z=%f",
            //          transformed_pose.pose.position.x,
            //          transformed_pose.pose.position.y,
            //          transformed_pose.pose.position.z);
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

            if (isCoarse)
            {
                result->fruit_data.pose.orientation.x = 0;
                result->fruit_data.pose.orientation.y = 0;
                result->fruit_data.pose.orientation.z = 0;
                result->fruit_data.pose.orientation.w = 1;
                result->peduncle_data.pose.orientation.x = 0;
                result->peduncle_data.pose.orientation.y = 0;
                result->peduncle_data.pose.orientation.z = 0;
                result->peduncle_data.pose.orientation.w = 1;
            }
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

    void _logWithState(const std::string &message)
    {
        ROS_INFO_STREAM("[" << currentState << "]: " << message);
    }

    // Add this function to handle user input
    bool getUserConfirmation()
    {
        char input;
        std::cout << "Enter 'y' for yes or 'n' for no: ";
        std::cin >> input;

        if (input == 'y' || input == 'Y')
        {
            return true;
        }
        else if (input == 'n' || input == 'N')
        {
            return false;
        }
        else
        {
            std::cout << "Invalid input. Defaulting to 'no'." << std::endl;
            return false;
        }
    }

public:
    VADERStateMachine() : currentState(State::Home)
    {
        coarseEstimate = nullptr;
        coarseEstimateSub = n.subscribe("/fruit_coarse_pose", 10, coarseEstimateCallback);
        fineEstimate = nullptr;
        fineEstimateSub = n.subscribe("/fruit_fine_pose", 10, fineEstimateCallback);

        planClient = n.serviceClient<vader_msgs::SingleArmPlanRequest>("vader_plan");
        execClient = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("vader_exec");
        moveToStorageClient = n.serviceClient<vader_msgs::MoveToStorageRequest>("move_to_storage");
        goHomeClient = n.serviceClient<vader_msgs::GoHomeRequest>("go_home");

        storageBinLocation = new geometry_msgs::Pose();
        std::string bin_xyz;
        if (n.getParam("/bin_xyz", bin_xyz))
        {
            std::stringstream ss(bin_xyz);
            ss >> storageBinLocation->position.x >> storageBinLocation->position.y >> storageBinLocation->position.z;
            ROS_INFO_STREAM("Loaded bin_xyz: " << bin_xyz);
        }
        else
        {
            ROS_WARN("Failed to get param '/bin_xyz', using default values");
        }
        storageBinLocation->orientation.x = 0;
        storageBinLocation->orientation.y = 1;
        storageBinLocation->orientation.z = 0;
        storageBinLocation->orientation.w = 0;
        // TODO update

        gripperCommandPub = n.advertise<vader_msgs::GripperCommand>("/gripper_command", 10);
    }

    void setCoarsePoseEstimate(const vader_msgs::Pepper::ConstPtr &msg)
    {
        if (currentState == State::WaitForCoarseEstimate)
        {
            coarseEstimate = new vader_msgs::Pepper();
            coarseEstimate->header = msg->header;
            _transformFromCameraFrameIntoRobotFrame(msg, coarseEstimate, true);
            _logWithState("Coarse estimate received");
        }
    }

    void setFinePoseEstimate(const vader_msgs::Pepper::ConstPtr &msg)
    {
        if (currentState == State::WaitForFineEstimate)
        {
            fineEstimate = new vader_msgs::Pepper();
            fineEstimate->header = msg->header;
            _transformFromCameraFrameIntoRobotFrame(msg, fineEstimate, false);
            _logWithState("Fine estimate received");
        }
    }

    void execute()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            switch (currentState)
            {
            case State::Home:
            {
                ros::Duration(10.0).sleep();
                int NUM_EXEC_TRIES = 1;
                bool success = false;
                vader_msgs::GoHomeRequest request;
                request.request.exec = true;
                for (int i = 0; i < NUM_EXEC_TRIES; i++)
                {
                    if (goHomeClient.call(request))
                    {
                        if (request.response.result == 1)
                        {
                            _logWithState("Execution successful");
                            success = true;
                            break;
                        }
                    }
                }
                if (success)
                {
                    _logWithState("Move to bin execution successful, switching states");
                    currentState = State::WaitForCoarseEstimate;
                }
                else
                {
                    _logWithState("Move to bin execution failed");
                    currentState = State::Error;
                }
                break;
            }
            case State::WaitForCoarseEstimate:
            {
                if (coarseEstimate != nullptr)
                {
                    _logWithState("Coarse estimate received, switching states");
                    currentState = State::PlanGripperToPregrasp;
                }
                else
                {
                    _logWithState("Waiting for coarse estimate");
                }
                break;
            }
            case State::PlanGripperToPregrasp:
            {
                int NUM_PLAN_TRIES = 3;
                bool success = false;
                vader_msgs::SingleArmPlanRequest request;
                request.request.mode = request.request.GRIPPER_PREGRASP_PLAN;
                request.request.pepper = *coarseEstimate;
                request.request.reserve_dist = 0.1;
                request.request.gripper_camera_rotation = request.request.GRIPPER_DO_ROTATE_CAMERA;
                for (int i = 0; i < NUM_PLAN_TRIES; i++)
                {
                    if (planClient.call(request))
                    {
                        if (request.response.result == 1)
                        {
                            _logWithState("Planning successful");
                            success = true;
                            break;
                        }
                    }
                }
                if (success)
                {
                    _logWithState("Gripper pregrasp planning successful, switching states");
                    currentState = State::MoveGripperToPregrasp;
                }
                else
                {
                    _logWithState("Gripper pregrasp planning failed");
                    // currentState = State::Error;
                    currentState = State::PlanGripperToPregrasp;
                }
                break;
            }
            case State::MoveGripperToPregrasp:
            {
                int NUM_EXEC_TRIES = 1;
                bool success = false;
                vader_msgs::SingleArmExecutionRequest request;
                request.request.mode = request.request.GRIPPER_PREGRASP_EXEC;
                for (int i = 0; i < NUM_EXEC_TRIES; i++)
                {
                    if (execClient.call(request))
                    {
                        if (request.response.result == 1)
                        {
                            _logWithState("Execution successful");
                            success = true;
                            break;
                        }
                    }
                }
                if (success)
                {
                    _logWithState("Gripper pregrasp execution successful, switching states");
                    currentState = State::WaitForFineEstimate;
                }
                else
                {
                    _logWithState("Gripper pregrasp execution failed");
                    currentState = State::Error;
                }
                break;
            }
            case State::WaitForFineEstimate:
            {
                if (fineEstimate != nullptr)
                {
                    _logWithState("Fine estimate received, switching states");
                    currentState = State::PlanGripperToGrasp;
                }
                else
                {
                    _logWithState("Waiting for fine estimate");
                }
                break;
            }
            case State::PlanGripperToGrasp:
            {
                ros::Duration(3.0).sleep();
                int NUM_PLAN_TRIES = 3;
                bool success = false;
                vader_msgs::SingleArmPlanRequest request;
                request.request.mode = request.request.GRIPPER_GRASP_PLAN;
                request.request.reserve_dist = 0.1;
                request.request.pepper = *fineEstimate;
                for (int i = 0; i < NUM_PLAN_TRIES; i++)
                {
                    if (planClient.call(request))
                    {
                        if (request.response.result == 1)
                        {
                            _logWithState("Planning successful");
                            success = true;
                            break;
                        }
                    }
                }
                if (success)
                {
                    _logWithState("Gripper grasp planning successful, switching states");
                    currentState = State::MoveGripperToGrasp;
                }
                else
                {
                    _logWithState("Gripper grasp planning failed");
                    currentState = State::WaitForFineEstimate;
                }
                break;
            }
            case State::MoveGripperToGrasp:
            {

                // bool userConfirmed = getUserConfirmation(); // Get user input
                // if (userConfirmed) {
                //     _logWithState("Confirmed");
                // } else {
                //     _logWithState("Replanning");
                //     currentState = State::PlanGripperToGrasp;
                //     break;
                // }
                int NUM_EXEC_TRIES = 1;
                bool success = false;
                vader_msgs::SingleArmExecutionRequest request;
                request.request.mode = request.request.GRIPPER_GRASP_EXEC;
                for (int i = 0; i < NUM_EXEC_TRIES; i++)
                {
                    if (execClient.call(request))
                    {
                        if (request.response.result == 1)
                        {
                            _logWithState("Execution successful");
                            success = true;
                            break;
                        }
                    }
                }
                if (success)
                {
                    _logWithState("Gripper grasp execution successful, switching states");
                    currentState = State::GripperGrasp;
                }
                else
                {
                    _logWithState("Gripper grasp execution failed");
                    currentState = State::Error;
                }
                break;
            }
            case State::GripperGrasp:
            {
                _logWithState("Grasping fruit");
                _sendGripperCommand(0);
                ros::Duration(5.0).sleep();
                // _sendGripperCommand(100);
                // ros::Duration(1.0).sleep();
                currentState = State::PlanAndMoveToBin;
                break;
            }
            case State::PlanAndMoveToBin:
            {
                int NUM_EXEC_TRIES = 3;
                bool success = false;
                vader_msgs::MoveToStorageRequest request;
                request.request.reserve_dist = 0.2;
                request.request.binLocation = *storageBinLocation;
                for (int i = 0; i < NUM_EXEC_TRIES; i++)
                {
                    if (moveToStorageClient.call(request))
                    {
                        if (request.response.result == 1)
                        {
                            _logWithState("Execution successful");
                            success = true;
                            break;
                        }
                    }
                }
                if (success)
                {
                    _logWithState("Move to bin execution successful, switching states");
                    currentState = State::GripperRelease;
                }
                else
                {
                    _logWithState("Move to bin execution failed");
                    currentState = State::PlanAndMoveToBin;
                }
                break;
            }
            case State::GripperRelease:
            {
                _logWithState("Releasing gripper");
                _sendGripperCommand(100);
                currentState = State::Done;
                break;
            }
            case State::Done:
            {
                // ROS_INFO("Done");
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
    statemachine.execute();
    return 0;
}