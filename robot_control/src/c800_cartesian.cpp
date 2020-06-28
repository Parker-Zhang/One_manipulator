#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"c800_cartesian");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");
    std::string end_effector_link = arm.getEndEffectorLink();

    std::string reference_fram = "base_link";
    arm.setPoseReferenceFrame(reference_fram);

    arm.allowReplanning(true);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    //这的数据类型是什么？？
    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;

    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(start_pose);

    start_pose.position.z -= 0.2;
    // waypoints.push_back(start_pose);

    // start_pose.position.x -= 0.05;

    waypoints.push_back(start_pose);

    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;

    int maxtries = 100;
    int attempts = 0;

    while(fraction < 1.0 && attempts<maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
        attempts ++;
        if  (attempts % 10 ==0)
            ROS_INFO("still trying after %d attempts...",attempts);
    }

    if (fraction == 1)
    {
        ROS_INFO("path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        plan.trajectory_ = trajectory;

        arm.execute(plan);

        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown();
    return 0 ;

}