#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"c800_moveit_fk");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 这里指定规划组 别名（预先设置好的规划组名字）
    moveit::planning_interface::MoveGroupInterface arm("arm");

    //设置机械臂运动的允许误差值
    arm.setGoalJointTolerance(0.001);

    //设置允许的最大速度和加速度，0.2表示20%
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    //控制机械臂先回到初始的位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1); 

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;   
}