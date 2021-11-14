#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float32MultiArray.h"

#include <cmath>
#include <eigen3/Eigen/Dense>

Eigen::Matrix4f tfTransform_2_EigenMatrix(tf::StampedTransform transform)
{
    Eigen::Matrix4f mat;

    // mat <<
    // transform.getBasis().getRow(0).getX(),transform.getBasis().getRow(0).getY(),transform.getBasis().getRow(0).getZ(),transform.getOrigin().getX(),
    // transform.getBasis().getRow(1).getX(),transform.getBasis().getRow(1).getY(),transform.getBasis().getRow(1).getZ(),transform.getOrigin().getY(),
    // transform.getBasis().getRow(2).getX(),transform.getBasis().getRow(2).getY(),transform.getBasis().getRow(2).getZ(),transform.getOrigin().getZ(),
    // 0,0,0,1;

    tf::Vector3 pos = transform.getOrigin();
    tf::Matrix3x3 rot = transform.getBasis();

    mat << rot[0][0], rot[0][1], rot[0][2], pos[0], rot[1][0], rot[1][1],
        rot[1][2], pos[1], rot[2][0], rot[2][1], rot[2][2], pos[2], 0, 0, 0, 1;
    return mat;
}


float vertical_control(double targetHeight , double height ,double vel_z ){
	const float pParam = 2;
    const float iParam = 0;
    const float dParam = 0;
    const float vParam = -2;

    static float cumul = 0,lastE = 0;

    double e = (targetHeight - height);
    ROS_INFO("e_vertical: %f",e);
    cumul = cumul + e;
    float pv = pParam * e;

    float thrust = 5.45 + pv + iParam * cumul + dParam * (e - lastE) + vel_z * vParam;
    lastE = e;

	return thrust;
} 

void horizontal_control(Eigen::Matrix4f mat,Eigen::Vector3f sp,float &alphaCorr,float &betaCorr){
	Eigen::Vector4f vx, vy;
    vx << 1, 0, 0, 1;
    vy << 0, 1, 0, 1;

    // printing m matrix
    // ROS_INFO("%f %f %f %f",mat.coeff(0,0),mat.coeff(0,1),mat.coeff(0,2),mat.coeff(0,3));
    // ROS_INFO("%f %f %f %f",mat.coeff(1,0),mat.coeff(1,1),mat.coeff(1,2),mat.coeff(1,3));
    // ROS_INFO("%f %f %f %f",mat.coeff(2,0),mat.coeff(2,1),mat.coeff(2,2),mat.coeff(2,3));
    // ROS_INFO("%f %f %f %f",mat.coeff(3,0),mat.coeff(3,1),mat.coeff(3,2),mat.coeff(3,3));
    
    vx = mat * vx;
    vy = mat * vy;

    static double pAlphaE = 0,pBetaE = 0;

    float alphaE = vy[2] - mat.coeff(2, 3);
    alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - pAlphaE);
    pAlphaE = alphaE;

    float betaE = vx[2] - mat.coeff(2, 3);
    betaCorr = -0.25 * betaE - 2.1 * (betaE - pBetaE);
    pBetaE = betaE;

    static double cumulA = 0, cumulB = 0;

    cumulA = cumulA + sp[1]; 
	cumulB = cumulB + sp[0];

	float threshold = 0.5;

    static double psp1 = 0, psp2 = 0;
    if (abs(psp2) > threshold && abs(sp[2]) < threshold)
        cumulA = 0;

    if (abs(psp1) > threshold && abs(sp[1]) < threshold) 
		cumulB = 0;

    float kp = 0.08;
    float ki = 0.005;
    float kd = 3;

    alphaCorr = alphaCorr + kp * sp[1] + kd * (sp[1] - psp2) + ki *cumulA ;
	betaCorr  = betaCorr  - kp * sp[0] - kd * (sp[0] - psp1) - ki *cumulB;

	psp2 = sp[1];
    psp1 = sp[0];

    ROS_INFO("alphaE: %f betaE: %f", alphaE, betaE);
    // ROS_INFO("vy %f %f %f ",vy[0],vy[1],vy[2]);
    // ROS_INFO("vy[2]: %f m[2,3]: %f", vy[2], mat.coeff(2, 3));
}

// double rotational_control(float yaw , float des_yaw){
double rotational_control(float yaw_error){
	static float pYaw = 0;
    
	// double yaw_error = des_yaw - yaw;
    ROS_INFO("yaw_error: %f",yaw_error);
    double rotCorr = yaw_error * 0.1 + 2 * (yaw_error - pYaw);
    pYaw = yaw_error;

	return rotCorr;
}

void control(tf::StampedTransform transform, tf::StampedTransform vel_transform,
             tf::StampedTransform des_transform, float ( &vel )[4])
{
    // get relative position
    Eigen::Matrix4f mat = tfTransform_2_EigenMatrix(transform);

    Eigen::Vector3f pos = mat.block<3, 1>(0, 3);
    tf::Vector3 sp_tf = des_transform.getOrigin() - transform.getOrigin();
    Eigen::Vector3f sp = Eigen::Vector3f(sp_tf.getX(), sp_tf.getY(), sp_tf.getZ());
	tf::Vector3 l = vel_transform.getOrigin();

    double targetHeight = des_transform.getOrigin().getZ();
    double height = transform.getOrigin().getZ();
	double vel_z  = l[2];

	// Vertical control:
    float thrust = vertical_control(targetHeight, height, vel_z);

    // Horizontal control:
    float alphaCorr,betaCorr;
	horizontal_control(mat,sp,alphaCorr,betaCorr);

    // Rotational control:
    double des_euler[3],euler[3];
	des_transform.getBasis().getEulerYPR(des_euler[0], des_euler[1], des_euler[2] );
	transform.getBasis().getEulerYPR(euler[0], euler[1], euler[2] );
    
    double yaw_error =vel_transform.getRotation().getZ();
	double rotCorr = rotational_control(yaw_error);
    
    vel[0] = thrust * (1 - alphaCorr + betaCorr + rotCorr);
	vel[1] = thrust * (1 - alphaCorr - betaCorr - rotCorr); 
	vel[2] = thrust * (1 + alphaCorr - betaCorr + rotCorr); 
	vel[3] = thrust * (1 + alphaCorr + betaCorr - rotCorr);

	// ROS_INFO("Thrust: %f  alphaCorr: %f  betaCorr: %f rotCorr: %f",thrust,alphaCorr,betaCorr,rotCorr);
    ROS_INFO("sp: %f %f %f", sp[0], sp[1], sp[2]);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_listener");
    // euler from quaternion
    tf::Quaternion q;

    ros::NodeHandle node;

    tf::TransformListener listener;

    ros::Publisher rpm_pub = node.advertise<std_msgs::Float32MultiArray>("rotors_RPM", 1000);
    
    ros::Rate rate(10.0);
    while (node.ok())
    {
        tf::StampedTransform transform_pos, transform_vel;
        try
        {
            listener.lookupTransform("/world", "/drone_0_pos", ros::Time(0),
                                     transform_pos);
            listener.lookupTransform("/world", "/drone_0_vel", ros::Time(0),
                                     transform_vel);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        tf::Vector3 pos = transform_pos.getOrigin();
        // ROS_INFO("x: %f, y: %f, z: %f", pos.x(), pos.y(), pos.z());

        tf::Matrix3x3 rot = transform_pos.getBasis();
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);

        pos = transform_vel.getOrigin();
       
        tf::StampedTransform des_transform;
        des_transform.setOrigin(tf::Vector3(-0.62500011920929, 0.64999997615814, 1.2500002384186));
        tf::Quaternion q;
        q.setRPY(-0.0002207071374869, -0.011167052201927, -0.00058256444754079);

        des_transform.setRotation(q);
        des_transform.stamp_ = ros::Time::now();

        float vel[4];
        control(transform_pos, transform_vel, des_transform,vel);

        std_msgs::Float32MultiArray msg;
        msg.data.push_back( (float) vel[0]);
        msg.data.push_back( (float) vel[1]);
        msg.data.push_back( (float) vel[2]);
        msg.data.push_back( (float) vel[3]);
        
        
        // Debugging
        // ROS_INFO("x_vel: %f, y_vel: %f, z_vel: %f", pos.x(), pos.y(), pos.z());
        // ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
	    //  ROS_INFO("Rotor RPMs: %f %f %f %f",vel[0],vel[1],vel[2],vel[3]);
        
        rpm_pub.publish(msg);

        rate.sleep();
    }
    return 0;
};