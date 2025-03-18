#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vader_msgs/Pepper.h"



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
    vader_msgs::Pepper::ConstPtr coarseEstimate;
    ros::NodeHandle n;
    ros::ServiceClient manipulationPlanningClient;
    ros::ServiceClient manipulationExecutionClient;

    public:
        VADERStateMachine() : currentState(State::Init){
            coarseEstimate = nullptr;
            ros::Subscriber coarseEstimateSub = n.subscribe("fruit_pose", 1000, VADERStateMachine::coarseEstimateCallback);
            manipulationPlanningClient = n.serviceClient<vader_msgs::Manipulation>("manipulationp");
            manipulationExecutionClient = n.serviceClient<vader_msgs::Manipulation>("manipulatione");
        }

        static void coarseEstimateCallback(const vader_msgs::Pepper::ConstPtr& msg) {
            if (currentState == State::Init) {
                std::cout << "Coarse estimate received" << std::endl;
                coarseEstimate = msg;
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
                        //send the coarse estimate to the manipulation module
                        ROS_INFO("Sending coarse estimate to manipulation module");
                        //something something service call for planning
                        bool success = false;
                        int NUM_TRIES = 3;
                        for (int i = 0; i < NUM_TRIES; i++) {
                            vader_msgs::Manipulation srv;
                            srv.request.pepper = *coarseEstimate;
                            if (manipulationPlanningClient.call(srv)) {
                                if (srv.response.success) {
                                    success = true;
                                    break;
                                }
                            }
                        }
                        if (success) {
                            ROS_INFO("Manipulation planning successful, switching states");
                            currentState = State::Manipulation;
                        } else {
                            ROS_INFO("Manipulation planning failed, switching states");
                            currentState = State::Error;
                        }
                        break;
                    case State::Manipulation:
                        //Send service call for manipulation to execute
                        ROS_INFO("Sending execute command to manipulation module");
                        //something something service call for execution
                        vader_msgs::Manipulation srv;
                        srv.request.pepper = *coarseEstimate;
                        if (manipulationExecutionClient.call(srv)) {
                            if (srv.response.success) {
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
                    case State::Done:
                        ROS_INFO("Done");
                        break;
                    case State::Error:
                        ROS_INFO("Error");
                        break;

                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

};

int main(int argc, char **argv) {
    VADERStateMachine sm;

    ros::init(argc, argv, "vader_hri");
    sm.execute();
    return 0;
}