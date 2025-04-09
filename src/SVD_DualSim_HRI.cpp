#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"
#include "vader_msgs/SingleArmPlanRequest.h"
#include "vader_msgs/SingleArmExecutionRequest.h"
#include "vader_msgs/GripperCommand.h"
#include "vader_msgs/CutterCommand.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>

static void coarseEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg);
static void fineEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg);

class VADERStateMachine {
    private:
    enum State {
        // ----------Coarse-Pregrasp stage-----------
        WaitForCoarseEstimate,
        PlanGripperToPregrasp,
        MoveGripperToPregrasp,
        // ----------Fine-Grasp stage-----------
        WaitForFineEstimate,
        PlanGripperToGrasp,
        MoveGripperToGrasp,
        PlanCutterToGrasp,
        MoveCutterToGrasp,
        // ------------Harvest stage-----------
        GripperGrasp,
        CutterGrasp,
        // -----------Finish and cleanup-----------
        PlanAndMoveToBin,
        GripperRelease,
        Done,
        Error
    };

    State currentState;
    ros::NodeHandle n;

    //Perception subscriptions and data
    ros::Subscriber coarseEstimateSub, fineEstimateSub;
    vader_msgs::Pepper* coarseEstimate;
    vader_msgs::Pepper* fineEstimate;
    vader_msgs::Pepper* storageBinLocation; //only the fruit pose is used to designate storage bin location.

    //Plan/Exec clients connecting to planner
    ros::ServiceClient pregraspGripperPlanClient, pregraspGripperExecClient;
    ros::ServiceClient pregraspCutterPlanClient, pregraspCutterExecClient;
    ros::ServiceClient graspGripperPlanClient, graspGripperExecClient;
    ros::ServiceClient graspCutterPlanClient, graspCutterExecClient;
    ros::ServiceClient planAndMoveToBinClient;

    //End effector talking to dynamixel node
    ros::Publisher gripperCommandPub, cutterCommandPub;

    /*
    Input: cameraFrameMsg containing the pose of the fruit in the camera frame from the coarse/fine pepper estimate
    Output: result containing the pose of the fruit in the world frame using TF2.
    */
    void _transformFromCameraFrameIntoRobotFrame(const vader_msgs::Pepper::ConstPtr& cameraFrameMsg, vader_msgs::Pepper* result) {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer); 
        geometry_msgs::PoseStamped fruit_pose;
        fruit_pose.pose.position.x = -cameraFrameMsg->fruit_data.pose.position.y; 
        fruit_pose.pose.position.y = cameraFrameMsg->fruit_data.pose.position.x;
        fruit_pose.pose.position.z = cameraFrameMsg->fruit_data.pose.position.z;
        fruit_pose.pose.orientation.x = cameraFrameMsg->fruit_data.pose.orientation.x;
        fruit_pose.pose.orientation.y = cameraFrameMsg->fruit_data.pose.orientation.y;
        fruit_pose.pose.orientation.z = cameraFrameMsg->fruit_data.pose.orientation.z;
        fruit_pose.pose.orientation.w = cameraFrameMsg->fruit_data.pose.orientation.w;
        //TODO when peduncle data is published, do the same transform

        // ROS_INFO("CAMERA FRAME TF: x=%f, y=%f, z=%f, q_x=%f, q_y=%f, q_z=%f, q_w=%f",
        //     fruit_pose.pose.position.x, fruit_pose.pose.position.y, fruit_pose.pose.position.z,
        //     fruit_pose.pose.orientation.x, fruit_pose.pose.orientation.y, fruit_pose.pose.orientation.z, fruit_pose.pose.orientation.w );
        fruit_pose.header.frame_id = cameraFrameMsg->header.frame_id;

        try {
            geometry_msgs::PoseStamped transformed_pose = tf_buffer.transform(
                fruit_pose, 
                "link_base", 
                ros::Duration(3.0)
            );
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
            result->fruit_data.pose.orientation.w = transformed_pose.pose.orientation.w; //todo check if the orientation is correct
            result->fruit_data.shape.dimensions.resize(2);
            result->fruit_data.shape.dimensions[0] = cameraFrameMsg->fruit_data.shape.dimensions[0];
            result->fruit_data.shape.dimensions[1] = cameraFrameMsg->fruit_data.shape.dimensions[1];
          } 
        catch (tf2::TransformException &ex) {
            ROS_WARN("Transform failed: %s", ex.what());
        }
    }

    void _sendGripperCommand(int open_pct) {
        vader_msgs::GripperCommand gripperCommand;
        gripperCommand.open_pct = open_pct;
        gripperCommandPub.publish(gripperCommand);
        ROS_INFO_STREAM("Gripper command: open_pct=" << open_pct);
    }

    void _sendCutterCommand(int open_pct) {
        vader_msgs::CutterCommand cutterCommand;
        cutterCommand.open_pct = open_pct;
        cutterCommandPub.publish(cutterCommand);
        ROS_INFO_STREAM("Cutter command: open_pct=" << open_pct);
    }

    void _logWithState(const std::string& message) {
        ROS_INFO_STREAM("[" << currentState << "]: " << message);
    }

    bool _callPlannerService(ros::ServiceClient* client, const vader_msgs::Pepper& _estimate) {
        vader_msgs::SingleArmPlanRequest srv;
        srv.request.pepper = _estimate;
        if (client->call(srv)) {
            if (srv.response.result == 1) {
                _logWithState("Planning successful");
                return true;
            } else {
                _logWithState("Planning failed");
            }
        } else {
            _logWithState("Planner service call failed");
        }
        return false;
    }

    bool _callExecutorService(ros::ServiceClient* client) {
        vader_msgs::SingleArmExecutionRequest srv;
        srv.request.execute = 1;
        if (client->call(srv)) {
            if (srv.response.result == 1) {
                _logWithState("Execution successful");
                return true;
            } else {
                _logWithState("Execution failed");
            }
        } else {
            _logWithState("Executor service call failed");
        }
        return false;
    }

    public:
        VADERStateMachine() : currentState(State::WaitForCoarseEstimate){
            coarseEstimate = nullptr;
            coarseEstimateSub = n.subscribe("/fruit_coarse_pose", 10, coarseEstimateCallback);
            fineEstimate = nullptr;
            fineEstimateSub = n.subscribe("/fruit_fine_pose", 10, fineEstimateCallback);

            pregraspGripperPlanClient   = n.serviceClient<vader_msgs::SingleArmPlanRequest>("gripperArmPregraspPlan");
            pregraspGripperExecClient   = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("gripperArmPregraspExec");
            pregraspCutterPlanClient    = n.serviceClient<vader_msgs::SingleArmPlanRequest>("cutterArmPregraspPlan");
            pregraspCutterExecClient    = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("cutterArmPregraspExec");
            graspGripperPlanClient      = n.serviceClient<vader_msgs::SingleArmPlanRequest>("gripperArmGraspPlan");
            graspGripperExecClient      = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("gripperArmGraspExec");
            graspCutterPlanClient       = n.serviceClient<vader_msgs::SingleArmPlanRequest>("cutterArmGraspPlan");
            graspCutterExecClient       = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("cutterArmGraspExec");
            planAndMoveToBinClient      = n.serviceClient<vader_msgs::SingleArmPlanRequest>("planAndMoveToBin");

            storageBinLocation = new vader_msgs::Pepper();
            storageBinLocation->fruit_data.pose.position.x = 0.15;
            storageBinLocation->fruit_data.pose.position.y = -0.4;
            storageBinLocation->fruit_data.pose.position.z = 0.25;
            storageBinLocation->fruit_data.pose.orientation.x = 1.0;
            storageBinLocation->fruit_data.pose.orientation.y = 0.0;
            storageBinLocation->fruit_data.pose.orientation.z = 0.0;
            storageBinLocation->fruit_data.pose.orientation.w = 0.0;

            gripperCommandPub = n.advertise<vader_msgs::GripperCommand>("/gripper_command", 10);
            cutterCommandPub = n.advertise<vader_msgs::CutterCommand>("/cutter_command", 10);
        }

        void setCoarsePoseEstimate(const vader_msgs::Pepper::ConstPtr& msg) {
            coarseEstimate = new vader_msgs::Pepper();
            coarseEstimate->header = msg->header;
            _transformFromCameraFrameIntoRobotFrame(msg, coarseEstimate);
            _logWithState("Coarse estimate received");
        }
    
        void setFinePoseEstimate(const vader_msgs::Pepper::ConstPtr& msg) {
            fineEstimate = new vader_msgs::Pepper();
            fineEstimate->header = msg->header;
            _transformFromCameraFrameIntoRobotFrame(msg, fineEstimate);
            _logWithState("Fine estimate received");
        }
    
        void execute() {
            ros::Rate loop_rate(10);
            while (ros::ok()) {
                switch (currentState) {
                    case State::WaitForCoarseEstimate:
                    {
                        if (coarseEstimate != nullptr) {
                            _logWithState("Coarse estimate received, switching states");
                            currentState = State::PlanGripperToPregrasp;
                        } else {
                            _logWithState("Waiting for coarse estimate");
                        }
                        break;
                    }
                    case State::PlanGripperToPregrasp:
                    {
                        int NUM_PLAN_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_PLAN_TRIES; i++) {
                            if (_callPlannerService(&pregraspGripperPlanClient, *coarseEstimate)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Gripper pregrasp planning successful, switching states");
                            currentState = State::MoveGripperToPregrasp;
                        } else {
                            _logWithState("Gripper pregrasp planning failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::MoveGripperToPregrasp:
                    {
                        int NUM_EXEC_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (_callExecutorService(&pregraspGripperExecClient)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Gripper pregrasp execution successful, switching states");
                            currentState = State::WaitForFineEstimate;
                        } else {
                            _logWithState("Gripper pregrasp execution failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::WaitForFineEstimate:
                    {
                        if (fineEstimate != nullptr) {
                            _logWithState("Fine estimate received, switching states");
                            currentState = State::PlanGripperToGrasp;
                        } else {
                            _logWithState("Waiting for fine estimate");
                        }
                        break;
                    }
                    case State::PlanGripperToGrasp:
                    {
                        int NUM_PLAN_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_PLAN_TRIES; i++) {
                            if (_callPlannerService(&graspGripperPlanClient, *fineEstimate)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Gripper grasp planning successful, switching states");
                            currentState = State::MoveGripperToGrasp;
                        } else {
                            _logWithState("Gripper grasp planning failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::MoveGripperToGrasp:
                    {
                        int NUM_EXEC_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (_callExecutorService(&graspGripperExecClient)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Gripper grasp execution successful, switching states");
                            currentState = State::PlanCutterToGrasp;
                        } else {
                            _logWithState("Gripper grasp execution failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::PlanCutterToGrasp:
                    {
                        int NUM_PLAN_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_PLAN_TRIES; i++) {
                            if (_callPlannerService(&pregraspCutterPlanClient, *fineEstimate)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Cutter pregrasp planning successful, switching states");
                            currentState = State::MoveCutterToGrasp;
                        } else {
                            _logWithState("Cutter pregrasp planning failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::MoveCutterToGrasp:
                    {
                        int NUM_EXEC_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (_callExecutorService(&pregraspCutterExecClient)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Cutter pregrasp execution successful, switching states");
                            currentState = State::GripperGrasp;
                        } else {
                            _logWithState("Cutter pregrasp execution failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::GripperGrasp:
                    {
                        _logWithState("Grasping fruit");
                        _sendGripperCommand(0);
                        currentState = State::CutterGrasp;
                        break;
                    }
                    case State::CutterGrasp:
                    {
                        int NUM_CUTS = 2;
                        _logWithState("Cutting peduncle");
                        for (int i = 0; i < NUM_CUTS; i++) {
                            _sendCutterCommand(0);
                            ros::Duration(1.0).sleep();
                            _sendCutterCommand(100);
                            ros::Duration(1.0).sleep();
                        }

                        currentState = State::PlanAndMoveToBin;
                        break;
                    }
                    case State::PlanAndMoveToBin:
                    {
                        int NUM_EXEC_TRIES = 3;
                        bool success = false;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (_callPlannerService(&planAndMoveToBinClient, *storageBinLocation)) {
                                success = true;
                                break;
                            }
                        }
                        if (success) {
                            _logWithState("Cutter pregrasp execution successful, switching states");
                            currentState = State::GripperRelease;
                        } else {
                            _logWithState("Cutter pregrasp execution failed");
                            currentState = State::Error;
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
                        ROS_INFO("Done");
                        break;
                    }
                    case State::Error:
                    {
                        ROS_INFO("Error");
                        break;
                    }

                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

};


VADERStateMachine* sm = nullptr;

static void coarseEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg) {
    if (sm != nullptr) {
        sm->setCoarsePoseEstimate(msg);
    } else {
        ROS_ERROR("State machine is not initialized");
    }
}

static void fineEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg) {
    if (sm != nullptr) {
        sm->setFinePoseEstimate(msg);
    } else {
        ROS_ERROR("State machine is not initialized");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vader_hri");
    VADERStateMachine statemachine = VADERStateMachine();
    sm = &statemachine;
    statemachine.execute();
    return 0;
}