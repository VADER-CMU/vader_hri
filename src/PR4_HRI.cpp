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
                        //wait for a perception coarse estimate to be published
                        ROS_INFO("Waiting for coarse estimate");
                        if (estimate != nullptr) {
                            ROS_INFO("Coarse estimate received, switching states");
                            currentState = State::PlanToPregrasp;
                        }
                        break;
                    }
                    case State::PlanToPregrasp:
                    {
                        //send the coarse estimate to the manipulation module
                        ROS_INFO("Sending coarse estimate to manipulation module");
                        //something something service call for planning
                        bool success = false;
                        int NUM_TRIES = 3;
                        for (int i = 0; i < NUM_TRIES; i++) {
                            vader_msgs::SingleArmPlanRequest srv;
                            srv.request.pepper = *estimate;
                            if (singleArmPlanClient.call(srv)) {
                                if (srv.response.result == 1) {
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            ROS_INFO("Manipulation planning successful, switching states");
                            currentState = State::MoveToPregrasp;
                            // currentState = State::WaitForCoarseEstimate;
                        } else {
                            ROS_INFO("Manipulation planning failed");
                            // currentState = State::Error;
                        }
                        break;
                    }
                    case State::MoveToPregrasp:
                    {
                        //Send service call for manipulation to execute
                        ROS_INFO("Sending execute command to manipulation module");
                        //something something service call for execution
                        vader_msgs::SingleArmExecutionRequest srv;
                        srv.request.execute = 1;
                        if (singleArmExecClient.call(srv)) {
                            if (srv.response.result == 1) {
                                ROS_INFO("Manipulation execution successful, switching states");
                                currentState = State::WaitForFineEstimate;
                                // estimate = nullptr;
                            } else {
                                ROS_INFO("Manipulation execution failed, switching states");
                                currentState = State::Error;
                            }
                        } else {
                            ROS_INFO("Manipulation execution failed, switching states");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::WaitForFineEstimate: 
                    {
                        //wait for a perception fine estimate to be published
                        // ROS_INFO("Waiting for fine estimate");
                        // if (estimate != nullptr) {
                        //     ROS_INFO("Fine estimate received, switching states");
                            currentState = State::PlanToGrasp;
                        // }
                        // break;
                    }
                    case State::PlanToGrasp:
                    {
                        //send the fine estimate to the manipulation module
                        ROS_INFO("Sending fine estimate to manipulation module");
                        //something something service call for planning
                        bool success = false;
                        int NUM_TRIES = 3;
                        for (int i = 0; i < NUM_TRIES; i++) {
                            vader_msgs::SingleArmPlanRequest srv;
                            srv.request.pepper = *estimate;
                            if (singleArmPlanFinalClient.call(srv)) {
                                if (srv.response.result == 1) {
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            ROS_INFO("Manipulation planning successful, switching states");
                            currentState = State::MoveToGrasp;
                        } else {
                            ROS_INFO("Manipulation planning failed");
                            // currentState = State::Error;
                        }
                        break;
                    }
                    case State::MoveToGrasp:
                    {
                        //Send service call for manipulation to execute
                        ROS_INFO("Sending execute command to manipulation module");
                        //something something service call for execution
                        vader_msgs::SingleArmExecutionRequest srv;
                        srv.request.execute = 1;
                        if (singleArmExecFinalClient.call(srv)) {
                            if (srv.response.result == 1) {
                                ROS_INFO("Manipulation execution successful, switching states");
                                currentState = State::GripperHarvest;
                            } else {
                                ROS_INFO("Manipulation execution failed, switching states");
                                currentState = State::Error;
                            }
                        } else {
                            ROS_INFO("Manipulation execution failed, switching states");
                            currentState = State::Error;
                        }
                        break;
                    }
                    case State::GripperHarvest:
                    {
                        vader_msgs::GripperCommand gripperCommand;
                        gripperCommand.open_pct = 0;
                        gripperCommandPub.publish(gripperCommand);
                        ROS_INFO("Gripper close command published, waiting ten seconds"); //this is when we manually cut wire
                        ros::Duration(10.0).sleep();
                        currentState = State::GripperRelease;
                        break;
                    }
                    // case State::PlanAndMoveToBin:
                    // {
                    //     vader_msgs::Pepper* bin_pepper = new vader_msgs::Pepper();
                    //     bin_pepper->fruit_data.pose.position.x = 0.5;
                    //     bin_pepper->fruit_data.pose.position.y = 0.5;
                    //     bin_pepper->fruit_data.pose.position.z = 0.5;
                    //     bin_pepper->fruit_data.pose.orientation.x = 1.0;
                    //     bin_pepper->fruit_data.pose.orientation.y = 0.0;
                    //     bin_pepper->fruit_data.pose.orientation.z = 0.0;
                    //     bin_pepper->fruit_data.pose.orientation.w = 0.0;
                        //TODO which message do we use
                    // }
                    case State::GripperRelease:
                    {
                        vader_msgs::GripperCommand gripperCommand;
                        gripperCommand.open_pct = 100;
                        gripperCommandPub.publish(gripperCommand);
                        ROS_INFO("Gripper open command published");
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