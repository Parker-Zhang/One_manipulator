#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"c800_collision");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    
    moveit::planning_interface::MoveGroupInterface arm("arm");
    //声明情景
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit_msgs::PlanningScene planning_scene;

    // 首先声明一个障碍物体，参考坐标系，id等属性
    moveit_msgs::CollisionObject collision_object;
    // collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.header.frame_id = "base_link";
    collision_object.id = "box1";

    // 设置障碍物的外形、尺寸
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;//还有 CYLINDER等
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    // 设置障碍物的位置
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.8;

    //将障碍物的属性、位置加入到障碍物的实例中
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    // planning_scene.world.collision_objects.push_back(collision_object);
    // planning_scene.is_diff = true;
    // planning_scene_diff_publisher.publish(planning_scene);

    sleep(2);
    //将障碍物attach到机器人上
    arm.attachObject(collision_object.id);

    //机器人运动
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

    //机器人放下障碍物
    arm.detachObject(collision_object.id);

    //机器人回到原位
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // //将障碍物移除情景
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    ros::shutdown(); 

    return 0;
}
