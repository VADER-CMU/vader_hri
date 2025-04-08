#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"
#include "vader_msgs/SingleArmPlanRequest.h"
#include "vader_msgs/SingleArmExecutionRequest.h"

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
        Init,
        CoarseEstimate,
        Manipulation,
        Done,
        Error
    };
    State currentState;
    vader_msgs::Pepper* coarseEstimate;
    ros::NodeHandle n;
    ros::ServiceClient singleArmPlanClient;
    ros::ServiceClient singleArmExecClient;
    ros::Subscriber coarseEstimateSub;

    public:
        VADERStateMachine() : currentState(State::Init){
            coarseEstimate = nullptr;
            coarseEstimateSub = n.subscribe("/fruit_pose", 10, poseEstimateCallback);
            singleArmPlanClient = n.serviceClient<vader_msgs::SingleArmPlanRequest>("singleArmPlan"); //TODO get names
            singleArmExecClient = n.serviceClient<vader_msgs::SingleArmExecutionRequest>("singleArmExec"); //TODO get names
        }

        void setPoseEstimate(const vader_msgs::Pepper::ConstPtr& msg) {
            if(currentState == State::Init) {
            // if(currentState == State::Init){
            //     coarseEstimate = msg;
            //     ROS_INFO("Coarse estimate received, switching states");
            //     currentState = State::CoarseEstimate;
            // }

                tf2_ros::Buffer tf_buffer;
                tf2_ros::TransformListener tf_listener(tf_buffer); 
                ROS_INFO("Coarse estimate received, switching states");
                currentState = State::CoarseEstimate;
                geometry_msgs::PoseStamped fruit_pose;
                fruit_pose.pose.position.x = msg->fruit_data.pose.position.x;
                fruit_pose.pose.position.y = msg->fruit_data.pose.position.y;
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
                    // ROS_INFO("Transformed pose: x=%f, y=%f", 
                    //          transformed_pose.pose.position.x,
                    //          transformed_pose.pose.position.y);
                    coarseEstimate = new vader_msgs::Pepper();
                    coarseEstimate->fruit_data.pose.position.x = transformed_pose.pose.position.x;
                    coarseEstimate->fruit_data.pose.position.y = transformed_pose.pose.position.y;
                    coarseEstimate->fruit_data.pose.position.z = transformed_pose.pose.position.z;
                    coarseEstimate->fruit_data.pose.orientation.x = 0;// fruit_pose.pose.orientation.x;
                    coarseEstimate->fruit_data.pose.orientation.y = 0;//fruit_pose.pose.orientation.y;
                    coarseEstimate->fruit_data.pose.orientation.z = 0;//fruit_pose.pose.orientation.z;
                    coarseEstimate->fruit_data.pose.orientation.w = 1;//fruit_pose.pose.orientation.w;
                    coarseEstimate->fruit_data.shape.dimensions.resize(2);
                    coarseEstimate->fruit_data.shape.dimensions[0] = msg->fruit_data.shape.dimensions[0];
                    coarseEstimate->fruit_data.shape.dimensions[1] = msg->fruit_data.shape.dimensions[1];
                  } 
                catch (tf2::TransformException &ex) {
                    ROS_WARN("Transform failed: %s", ex.what());
                }

                
            }
        }

        void execute() {
            ros::Rate loop_rate(10);
            while (ros::ok()) {
                switch (currentState) {
                    case State::Init:
                        //wait for a perception coarse estimate to be published
                        ROS_INFO("Waiting for coarse estimate");
                        if (coarseEstimate != nullptr) {
                            ROS_INFO("Coarse estimate received, switching states");
                            currentState = State::CoarseEstimate;
                        }
                        break;
                    case State::CoarseEstimate:
                    {
                        //send the coarse estimate to the manipulation module
                        ROS_INFO("Sending coarse estimate to manipulation module");
                        //something something service call for planning
                        bool success = false;
                        int NUM_TRIES = 3;
                        for (int i = 0; i < NUM_TRIES; i++) {
                            vader_msgs::SingleArmPlanRequest srv;
                            srv.request.pepper = *coarseEstimate;
                            if (singleArmPlanClient.call(srv)) {
                                if (srv.response.result == 1) {
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            ROS_INFO("Manipulation planning successful, switching states");
                            currentState = State::Manipulation;
                        } else {
                            ROS_INFO("Manipulation planning failed");
                            // currentState = State::Error;
                        }
                        break;
                    }
                    case State::Manipulation:
                    {
                        //Send service call for manipulation to execute
                        ROS_INFO("Sending execute command to manipulation module");
                        //something something service call for execution
                        vader_msgs::SingleArmExecutionRequest srv;
                        srv.request.execute = 1;
                        if (singleArmExecClient.call(srv)) {
                            if (srv.response.result == 1) {
                                ROS_INFO("Manipulation execution successful, switching states");
                                currentState = State::Done;
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
    std::cout << "Coarse estimate received" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vader_hri");
    VADERStateMachine statemachine = VADERStateMachine();
    sm = &statemachine;
    statemachine.execute();
    return 0;
}