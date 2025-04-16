#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"
#include "vader_msgs/SingleArmPlanRequest.h"
#include "vader_msgs/SingleArmExecutionRequest.h"
#include "vader_msgs/GripperCommand.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>

static void poseEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg);

class VADERStateMachine {
    private:
    enum State {
        WaitForCoarseEstimate,
        PlanToPregrasp,
        MoveToPregrasp,
        WaitForFineEstimate,
        PlanToGrasp,
        MoveToGrasp,
        GripperHarvest,
        PlanAndMoveToBin,
        GripperRelease,
        Done,
        Error
    };
    State currentState;
    vader_msgs::Pepper* estimate;
    ros::NodeHandle n;
    ros::ServiceClient singleArmPlanClient;
    ros::ServiceClient singleArmExecClient;

    ros::ServiceClient singleArmPlanFinalClient;
    ros::ServiceClient singleArmExecFinalClient;
    ros::Subscriber coarseEstimateSub;
    ros::Publisher gripperCommandPub;

    public:
        VADERStateMachine() : currentState(State::WaitForCoarseEstimate){
            gripperCommandPub = n.advertise<vader_msgs::GripperCommand>("/gripper_command", 10);

            estimate = nullptr;
            coarseEstimateSub = n.subscribe("/fruit_coarse_pose", 10, poseEstimateCallback);
            singleArmPlanClient = n.serviceClient<vader_msgs::SingleArmPlanRequest>("singleArmPlan"); //TODO get names
            singleArmExecClient = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("singleArmExec"); //TODO get names

            singleArmPlanFinalClient = n.serviceClient<vader_msgs::SingleArmPlanRequest>("singleArmFinalPlan"); //TODO get names
            singleArmExecFinalClient = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("singleArmFinalExec"); //TODO get names
        }

        void setPoseEstimate(const vader_msgs::Pepper::ConstPtr& msg) {
            if(currentState == State::WaitForCoarseEstimate) {
            // if(currentState == State::WaitForCoarseEstimate){
            //     coarseEstimate = msg;
            //     ROS_INFO("Coarse estimate received, switching states");
            //     currentState = State::PlanToPregrasp;
            // }

                tf2_ros::Buffer tf_buffer;
                tf2_ros::TransformListener tf_listener(tf_buffer); 
                ROS_INFO("Coarse estimate received, switching states");
                // currentState = State::PlanToPregrasp;
                geometry_msgs::PoseStamped fruit_pose;
                fruit_pose.pose.position.x = -msg->fruit_data.pose.position.y; 
                fruit_pose.pose.position.y = msg->fruit_data.pose.position.x;
                fruit_pose.pose.position.z = msg->fruit_data.pose.position.z;
                fruit_pose.pose.orientation.x = msg->fruit_data.pose.orientation.x;
                fruit_pose.pose.orientation.y = msg->fruit_data.pose.orientation.y;
                fruit_pose.pose.orientation.z = msg->fruit_data.pose.orientation.z;
                fruit_pose.pose.orientation.w = msg->fruit_data.pose.orientation.w;

                ROS_INFO("CAMERA FRAME TF: x=%f, y=%f, z=%f, q_x=%f, q_y=%f, q_z=%f, q_w=%f",
                    fruit_pose.pose.position.x, fruit_pose.pose.position.y, fruit_pose.pose.position.z,
                    fruit_pose.pose.orientation.x, fruit_pose.pose.orientation.y, fruit_pose.pose.orientation.z, fruit_pose.pose.orientation.w );
                fruit_pose.header.frame_id = msg->header.frame_id;
                
                try {
                    geometry_msgs::PoseStamped transformed_pose = tf_buffer.transform(
                        fruit_pose, 
                        "link_base", 
                        ros::Duration(3.0)
                    );
                    ROS_INFO("Transformed pose: x=%f, y=%f, z=%f", 
                             transformed_pose.pose.position.x,
                             transformed_pose.pose.position.y,
                             transformed_pose.pose.position.z);
                    estimate = new vader_msgs::Pepper();
                    estimate->fruit_data.pose.position.x = transformed_pose.pose.position.x;
                    estimate->fruit_data.pose.position.y = transformed_pose.pose.position.y;
                    estimate->fruit_data.pose.position.z = transformed_pose.pose.position.z;
                    estimate->fruit_data.pose.orientation.x = 0;// fruit_pose.pose.orientation.x;
                    estimate->fruit_data.pose.orientation.y = 0;//fruit_pose.pose.orientation.y;
                    estimate->fruit_data.pose.orientation.z = 0;//fruit_pose.pose.orientation.z;
                    estimate->fruit_data.pose.orientation.w = 1;//fruit_pose.pose.orientation.w;
                    estimate->fruit_data.shape.dimensions.resize(2);
                    estimate->fruit_data.shape.dimensions[0] = msg->fruit_data.shape.dimensions[0];
                    estimate->fruit_data.shape.dimensions[1] = msg->fruit_data.shape.dimensions[1];
                  } 
                catch (tf2::TransformException &ex) {
                    ROS_WARN("Transform failed: %s", ex.what());
                }

            }
            // }else if(currentState == State::WaitForFineEstimate) {
    
            //         tf2_ros::Buffer tf_buffer;
            //         tf2_ros::TransformListener tf_listener(tf_buffer); 
            //         ROS_INFO("Fine estimate received, switching states");
            //         // currentState = State::PlanToGrasp;
            //         geometry_msgs::PoseStamped fruit_pose;
            //         fruit_pose.pose.position.x = -msg->fruit_data.pose.position.y; 
            //         fruit_pose.pose.position.y = msg->fruit_data.pose.position.x;
            //         fruit_pose.pose.position.z = msg->fruit_data.pose.position.z;
            //         fruit_pose.pose.orientation.x = msg->fruit_data.pose.orientation.x;
            //         fruit_pose.pose.orientation.y = msg->fruit_data.pose.orientation.y;
            //         fruit_pose.pose.orientation.z = msg->fruit_data.pose.orientation.z;
            //         fruit_pose.pose.orientation.w = msg->fruit_data.pose.orientation.w;
    
            //         ROS_INFO("CAMERA FRAME TF: x=%f, y=%f, z=%f, q_x=%f, q_y=%f, q_z=%f, q_w=%f",
            //             fruit_pose.pose.position.x, fruit_pose.pose.position.y, fruit_pose.pose.position.z,
            //             fruit_pose.pose.orientation.x, fruit_pose.pose.orientation.y, fruit_pose.pose.orientation.z, fruit_pose.pose.orientation.w );
            //         fruit_pose.header.frame_id = msg->header.frame_id;
                    
            //         try {
            //             geometry_msgs::PoseStamped transformed_pose = tf_buffer.transform(
            //                 fruit_pose, 
            //                 "link_base", 
            //                 ros::Duration(3.0)
            //             );
            //             ROS_INFO("Transformed pose: x=%f, y=%f, z=%f", 
            //                      transformed_pose.pose.position.x,
            //                      transformed_pose.pose.position.y,
            //                      transformed_pose.pose.position.z);
            //             estimate = new vader_msgs::Pepper();
            //             estimate->fruit_data.pose.position.x = transformed_pose.pose.position.x;
            //             estimate->fruit_data.pose.position.y = transformed_pose.pose.position.y;
            //             estimate->fruit_data.pose.position.z = transformed_pose.pose.position.z;
            //             estimate->fruit_data.pose.orientation.x = 0;// fruit_pose.pose.orientation.x;
            //             estimate->fruit_data.pose.orientation.y = 0;//fruit_pose.pose.orientation.y;
            //             estimate->fruit_data.pose.orientation.z = 0;//fruit_pose.pose.orientation.z;
            //             estimate->fruit_data.pose.orientation.w = 1;//fruit_pose.pose.orientation.w;
            //             estimate->fruit_data.shape.dimensions.resize(2);
            //             estimate->fruit_data.shape.dimensions[0] = msg->fruit_data.shape.dimensions[0];
            //             estimate->fruit_data.shape.dimensions[1] = msg->fruit_data.shape.dimensions[1];
            //           } 
            //         catch (tf2::TransformException &ex) {
            //             ROS_WARN("Transform failed: %s", ex.what());
            //         }
    
                    
            //     }
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
                        vader_msgs::BimanualPlanRequest request;
                        request.request.mode = request.request.GRIPPER_PREGRASP_PLAN;
                        request.request.pepper = *coarseEstimate;
                        request.request.reserve_dist = 0.2;
                        request.request.gripper_camera_rotation = request.request.GRIPPER_DO_ROTATE_CAMERA;
                        for (int i = 0; i < NUM_PLAN_TRIES; i++) {
                            if (planClient.call(request)){
                                if (request.response.result == 1) {
                                    _logWithState("Planning successful");
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            _logWithState("Gripper pregrasp planning successful, switching states");
                            currentState = State::MoveGripperToPregrasp;
                        } else {
                            _logWithState("Gripper pregrasp planning failed");
                            // currentState = State::Error;
                            currentState = State::WaitForCoarseEstimate;
                        }
                        break;
                    }
                    case State::MoveGripperToPregrasp:
                    {
                        int NUM_EXEC_TRIES = 3;
                        bool success = false;
                        vader_msgs::BimanualExecRequest request;
                        request.request.mode = request.request.GRIPPER_PREGRASP_EXEC;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (execClient.call(request)){
                                if (request.response.result == 1) {
                                    _logWithState("Execution successful");
                                    success = true;
                                    break;
                                }
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
                        vader_msgs::BimanualPlanRequest request;
                        request.request.mode = request.request.GRIPPER_GRASP_PLAN;
                        request.request.reserve_dist = 0.1;
                        request.request.pepper = *fineEstimate;
                        for (int i = 0; i < NUM_PLAN_TRIES; i++) {
                            if (planClient.call(request)){
                                if (request.response.result == 1) {
                                    _logWithState("Planning successful");
                                    success = true;
                                    break;
                                }
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
                        vader_msgs::BimanualExecRequest request;
                        request.request.mode = request.request.GRIPPER_GRASP_EXEC;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (execClient.call(request)){
                                if (request.response.result == 1) {
                                    _logWithState("Execution successful");
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            _logWithState("Gripper grasp execution successful, switching states");
                            currentState = State::GripperGrasp;
                        } else {
                            _logWithState("Gripper grasp execution failed");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::GripperGrasp:
                    {
                        _logWithState("Grasping fruit");
                        _sendGripperCommand(0);
                        ros::Duration(1.0).sleep();
                        currentState = State::PlanCutterToGrasp;
                        break;
                    }
                    // case State::PlanCutterToGrasp:
                    // {
                    //     int NUM_PLAN_TRIES = 1;
                    //     bool success = false;
                    //     vader_msgs::BimanualPlanRequest request;
                    //     request.request.mode = request.request.CUTTER_GRASP_PLAN;
                    //     request.request.reserve_dist = 0.2;//0.04;
                    //     request.request.pepper = *fineEstimate;
                    //     for (int i = 0; i < NUM_PLAN_TRIES; i++) {
                    //         if (planClient.call(request)){
                    //             if (request.response.result == 1) {
                    //                 _logWithState("Planning successful");
                    //                 success = true;
                    //                 break;
                    //             }
                    //         }
                    //     }
                    //     if (success) {
                    //         _logWithState("Cutter pregrasp planning successful, switching states");
                    //         currentState = State::MoveCutterToGrasp;
                    //     } else {
                    //         _logWithState("Cutter pregrasp planning failed");
                    //         currentState = State::Error;
                    //     }
                    //     break;
                    // }
                    // case State::MoveCutterToGrasp:
                    // {
                    //     int NUM_EXEC_TRIES = 3;
                    //     bool success = false;
                    //     vader_msgs::BimanualExecRequest request;
                    //     request.request.mode = request.request.CUTTER_GRASP_EXEC;
                    //     for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                    //         if (execClient.call(request)){
                    //             if (request.response.result == 1) {
                    //                 _logWithState("Execution successful");
                    //                 success = true;
                    //                 break;
                    //             }
                    //         }
                    //     }
                    //     if (success) {
                    //         _logWithState("Cutter pregrasp execution successful, switching states");
                    //         currentState = State::CutterGrasp;
                    //     } else {
                    //         _logWithState("Cutter pregrasp execution failed");
                    //         currentState = State::Error;
                    //     }
                    //     break;
                    // }
                    // case State::CutterGrasp:
                    // {
                    //     int NUM_CUTS = 2;
                    //     _logWithState("Cutting peduncle");
                    //     for (int i = 0; i < NUM_CUTS; i++) {
                    //         _sendCutterCommand(0);
                    //         ros::Duration(1.0).sleep();
                    //         _sendCutterCommand(100);
                    //         ros::Duration(1.0).sleep();
                    //     }

                    //     currentState = State::PlanAndMoveToBin;
                    //     break;
                    // }
                    case State::PlanAndMoveToBin:
                    {
                        int NUM_EXEC_TRIES = 3;
                        bool success = false;
                        vader_msgs::MoveToStorageRequest request;
                        request.request.reserve_dist = 0.2;
                        request.request.binLocation = *storageBinLocation;
                        for (int i = 0; i < NUM_EXEC_TRIES; i++) {
                            if (moveToStorageClient.call(request)){
                                if (request.response.result == 1) {
                                    _logWithState("Execution successful");
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            _logWithState("Move to bin execution successful, switching states");
                            currentState = State::GripperRelease;
                        } else {
                            _logWithState("Move to bin execution failed");
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


VADERStateMachine* sm = nullptr;

static void poseEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg) {
    sm->setPoseEstimate(msg);
    std::cout << "Pose estimate received" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vader_hri");
    VADERStateMachine statemachine = VADERStateMachine();
    sm = &statemachine;
    statemachine.execute();
    return 0;
}