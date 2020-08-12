#include <iostream>
#include <csignal>
#include <ros/ros.h>
#include <ros/package.h>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_listener.h>

class Controller {
    class robot {
        public:
            robot() : is_moving(false), tf_listener(tf_buffer), private_nh("~") {
                this->joint_state.init("iiwa");
                this->pose_state.init("iiwa");
                this->joint_command.init("iiwa");
                this->pose_command.init("iiwa");
            }

            bool isConnected() {
                return this->joint_state.isConnected() & this->pose_state.isConnected();
            }

            void startMotion() {
                std::lock_guard<std::mutex> lock(this->mutex);
                this->is_moving = true;
            }

            void finishMotion() {
                std::lock_guard<std::mutex> lock(this->mutex);
                this->is_moving = false;
            }

            bool isMoving() {
                std::lock_guard<std::mutex> lock(this->mutex);
                return this->is_moving;
            }

            void setPosition(const iiwa_msgs::JointPosition& command_position) {
                this->startMotion();
                joint_command.setPosition(command_position, [this](){return finishMotion();});
            }

            iiwa_msgs::JointPosition getPosition() {
                return joint_state.getPosition();
            }


            iiwa_msgs::JointPosition getInitialPosition() {
                std::vector<double> joints(7, 0.0);
                this->private_nh.getParam("initial_joints", joints);

                iiwa_msgs::JointPosition initial_position;
                initial_position.position.a1 = joints[0];
                initial_position.position.a2 = joints[1];
                initial_position.position.a3 = joints[2];
                initial_position.position.a4 = joints[3];
                initial_position.position.a5 = joints[4];
                initial_position.position.a6 = joints[5];
                initial_position.position.a7 = joints[6];
                initial_position.header.stamp = ros::Time();

                return initial_position;
            }

            void setPose(const geometry_msgs::PoseStamped& command_pose) {
                this->startMotion();
                pose_command.setPose(command_pose, [this](){return finishMotion();});
            }

            geometry_msgs::PoseStamped getPose() {
                return pose_state.getPose();
            }

            geometry_msgs::PoseStamped getPose(const std::string& source_frame) {
                geometry_msgs::TransformStamped transform_stamped;
                geometry_msgs::PoseStamped pose_stamped;

                try {
                    transform_stamped = this->tf_buffer.lookupTransform("iiwa_link_0", source_frame, ros::Time(0));
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s", ex.what());
                }

                pose_stamped.pose.position.x  = transform_stamped.transform.translation.x;
                pose_stamped.pose.position.y  = transform_stamped.transform.translation.y;
                pose_stamped.pose.position.z  = transform_stamped.transform.translation.z;
                pose_stamped.pose.orientation = transform_stamped.transform.rotation;
                pose_stamped.header.stamp = ros::Time();

                return pose_stamped;
            }

        private:
            std::mutex mutex;
            bool is_moving;
            iiwa_ros::state::JointPosition joint_state;
            iiwa_ros::state::CartesianPose pose_state;
            iiwa_ros::command::JointPosition joint_command;
            iiwa_ros::command::CartesianPose pose_command;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener;
            ros::NodeHandle private_nh;
    };

    class gripper {
        public:
            gripper() : gripper_action_client("/gripper", true) {
                ROS_INFO("Waiting for gripper action server");
                this->gripper_action_client.waitForServer(ros::Duration(60.0));
                ROS_INFO("Started gripper action server");
            }

            bool open() {
                control_msgs::GripperCommandGoal goal;
                goal.command.position = 0.085;
                goal.command.max_effort = 100.0;
                this->gripper_action_client.sendGoal(goal);
                bool result = this->gripper_action_client.waitForResult(ros::Duration(60.0));
                if (result) {
                    actionlib::SimpleClientGoalState state = this->gripper_action_client.getState();
                    ROS_INFO("Gripper action finished: %s", state.toString().c_str());
                }
                else {
                    ROS_WARN("Gripper action did not finish before the time out.");
                }
                return result;
            }

            bool close() {
                control_msgs::GripperCommandGoal goal;
                goal.command.position = 0.0;
                goal.command.max_effort = 100.0;
                this->gripper_action_client.sendGoal(goal);
                bool result = this->gripper_action_client.waitForResult(ros::Duration(60.0));
                if (result) {
                    actionlib::SimpleClientGoalState state = this->gripper_action_client.getState();
                    ROS_INFO("Gripper action finished: %s", state.toString().c_str());
                }
                else {
                    ROS_WARN("Gripper action did not finish before the time out.");
                }
                return result;
            }

        private:
            actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;
    };

    class jig {
        public:
            void jam() {
                ros::ServiceClient client = this->nh.serviceClient<std_srvs::SetBool>("control_pin");
                std_srvs::SetBool request;
                request.request.data = false;
                if (client.call(request)) {
                    std::cout << request.response.message << std::endl;
                }
            }

            void free() {
                ros::ServiceClient client = this->nh.serviceClient<std_srvs::SetBool>("control_pin");
                std_srvs::SetBool request;
                request.request.data = true;
                if (client.call(request)) {
                    std::cout << request.response.message << std::endl;
                }
            }
        private:
            ros::NodeHandle nh;
    };

    class camera {
        public:
            void capture() {
                pcl::PCLPointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<pcl::PCLPointCloud2>("/hand_camera/depth_registered/points");
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromPCLPointCloud2(*cloud_msg, *cloud);

                std::string package_path = ros::package::getPath("softjig_ros");
                std::stringstream ss;
                ss << package_path << "/pcd/" << cloud_msg->header.stamp << ".pcd";

                pcl::PCDWriter writer;
                writer.writeBinaryCompressed(ss.str(), *cloud);
            }
    };

    public:
        Controller() : iiwa(), robotiq(), jig(), realsense(), step(0) {}

        void goToNextStep() {
            this->step++;
        }

        void resetStep() {
            this->step = 0;
        }

        int getStep() {
            return this->step;
        }

        enum OperationState {
            Unactiveted,
            Initialized,
            MovedToPick,
            PickedObject,
            LiftedObject,
            MovedToPush,
            PushedToFix,
            FixedObject,
            ReleasedObject,
            LeavedToCaptureInitialPose,
            CapturedInitialPose,
            MovedToGrapple,
            Grappled,
            LeavedToCaptureFinalPose,
            CapturedFinalPose,
            FreedUpObject
        };

        robot iiwa;
        gripper robotiq;
        jig jig;
        camera realsense;

    private:
        int step;
};

extern "C" void signal_handler(int sig) {
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "softjig_controller_node");

    Controller controller;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGHUP, signal_handler);

    while (true) {
        ros::Duration(1.0).sleep();

        if (!controller.iiwa.isConnected()) {
            std::cerr << "iiwa is not connected :(" << std::endl;
            break;
        }

        if (controller.iiwa.isMoving()) {
            ROS_INFO("iiwa is moving :)");
            continue;
        }

        switch (controller.getStep()) {
            case Controller::Unactiveted:
            {
                ROS_INFO("Initialize");
                iiwa_msgs::JointPosition joint_position = controller.iiwa.getInitialPosition();
                controller.iiwa.setPosition(joint_position);
                break;
            }
            case Controller::Initialized:
            {
                ROS_INFO("Move to pick");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_pick_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::MovedToPick:
            {
                ROS_INFO("Pick object");
                controller.robotiq.close();
                break;
            }
            case Controller::PickedObject:
            {
                ROS_INFO("Lift object");
                iiwa_msgs::JointPosition joint_position = controller.iiwa.getInitialPosition();
                controller.iiwa.setPosition(joint_position);
                break;
            }
            case Controller::LiftedObject:
            {
                ROS_INFO("Move to push");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_place_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::MovedToPush:
            {
                ROS_INFO("Push object to fix");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_push_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::PushedToFix:
            {
                ROS_INFO("Fix object on the SoftJig");
                controller.jig.jam();
                break;
            }
            case Controller::FixedObject:
            {
                ROS_INFO("Release object");
                controller.robotiq.open();
                break;
            }
            case Controller::ReleasedObject:
            {
                ROS_INFO("Leave to capture initial pose");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_capture_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::LeavedToCaptureInitialPose:
            {
                ROS_INFO("Capture");
                controller.realsense.capture();
                ROS_INFO("Rotate flange");
                geometry_msgs::PoseStamped current_pose = controller.iiwa.getPose();
                geometry_msgs::PoseStamped target_pose = controller.iiwa.getPose("iiwa_grapple_start_pose");
                target_pose.pose.position.z = (current_pose.pose.position.z + target_pose.pose.position.z) / 2.0;
                controller.iiwa.setPose(target_pose);
                break;
            }
            case Controller::CapturedInitialPose:
            {
                ROS_INFO("Move to grapple object");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_grapple_start_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::MovedToGrapple:
            {
                ROS_INFO("Grapple");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_grapple_end_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::Grappled:
            {
                ROS_INFO("Leaved to capture grappled pose");
                geometry_msgs::PoseStamped pose_stamped = controller.iiwa.getPose("iiwa_capture_pose");
                controller.iiwa.setPose(pose_stamped);
                break;
            }
            case Controller::LeavedToCaptureFinalPose:
            {
                ROS_INFO("Capture");
                controller.realsense.capture();
                break;
            }
            case Controller::CapturedFinalPose:
            {
                ROS_INFO("Free up object");
                controller.jig.free();
                break;
            }
            case Controller::FreedUpObject:
            {
                ROS_INFO("Done");
                break;
            }
            default:
            {
                ros::shutdown();
                break;
            }
        }

        controller.goToNextStep();
    }

    spinner.stop();

    return 0;
}