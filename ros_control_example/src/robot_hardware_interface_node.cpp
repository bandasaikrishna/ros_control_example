#include <ros_control_example/robot_hardware_interface.h>

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=5;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
	client = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("/read_joint_state");
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    
	joint_name_="joint1";
    
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_name_, &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandle);

// Create position joint interface
    //hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_);
    //position_joint_interface_.registerHandle(jointPositionHandle);
    
// Create velocity joint interface
	//hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    //effort_joint_interface_.registerHandle(jointVelocityHandle);
    
// Create effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_);
	effort_joint_interface_.registerHandle(jointEffortHandle);
	
// Create Joint Limit interface   
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits("joint1", nh_, limits);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle);
	
/*
If you have more joints then,

    joint_name_= "joint2"
    
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle2(joint_name_, &joint_position_2, &joint_velocity_2, &joint_effort_2);
    joint_state_interface_.registerHandle(jointStateHandle);

//create the position/velocity/effort interface according to your actuator 
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_2);
    position_joint_interface_.registerHandle(jointPositionHandle2);
    
    hardware_interface::JointHandle jointVelocityHandle2(jointStateHandle2, &joint_velocity_command_2);
    effort_joint_interface_.registerHandle(jointVelocityHandle2);
    
    hardware_interface::JointHandle jointEffortHandle2(jointStateHandle2, &joint_effort_command_2);
	effort_joint_interface_.registerHandle(jointEffortHandle2);
	
//create joint limit interface.
    joint_limits_interface::getJointLimits("joint2", nh_, limits);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle2(jointEffortHandle2, limits);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle2);
	
	Repeat same for other joints
*/
	

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

	joint_read.request.req=1.0;
	
	if(client.call(joint_read))
	{
	    joint_position_ = angles::from_degrees(joint_read.response.res[0]);
	    joint_velocity_ = angles::from_degrees(joint_read.response.res[1]);
	    ROS_INFO("Current Pos: %.2f, Vel: %.2f",joint_position_,joint_velocity_);
/*
if more than one joint,
        get values for joint_position_2, joint_velocity_2,......
*/	    
	    
	}
	else
	{
	    joint_position_ = 0;
	    joint_velocity_ = 0;
	}
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    effortJointSaturationInterface.enforceLimits(elapsed_time);    
	joints_pub.data.clear();
	joints_pub.data.push_back(joint_effort_command_);
	
/*
if more than one joint,
    publish values for joint_effort_command_2,......
*/	
	
	ROS_INFO("PWM Cmd: %.2f",joint_effort_command_);
	pub.publish(joints_pub);
		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
