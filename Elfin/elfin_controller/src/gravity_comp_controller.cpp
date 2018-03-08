#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
#define JointMax 6

namespace elfin_controller{

	class GravityCompController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{
    		if (!n.getParam("joints", joint_names_))
			{
				ROS_ERROR("Could not find joint name");
				return false;
    		}
			n_joints_ = joint_names_.size();
			tau.resize(n_joints_);

			if(n_joints_ == 0)
			{
				ROS_ERROR("List of joint names is empty.");
				return false;
			}

			urdf::Model urdf;
			if (!urdf.initParam("robot_description"))
			{
				ROS_ERROR("Failed to parse urdf file");
            	return false;
			}

			for(int i=0; i<n_joints_; i++)
			{
				try
				{
					joints_.push_back(hw->getHandle(joint_names_[i]));
				}
				catch (const hardware_interface::HardwareInterfaceException& e)
				{
					ROS_ERROR_STREAM("Exception thron: " << e.what());
					return false;
				}

				urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
				if (!joint_urdf)
				{
					ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                	return false;
				}
				joint_urdfs_.push_back(joint_urdf); 
			}

			// kdl parser
			if (!kdl_parser::treeFromUrdfModel(urdf, tree_)){
				ROS_ERROR("Failed to construct kdl tree");
				return false;
			}

			// kdl chain
			if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
			{
				ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
				ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
				ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
				ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
				ROS_ERROR_STREAM("  The segments are:");

				KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            	KDL::SegmentMap::iterator it;

            	for( it=segment_map.begin(); it != segment_map.end(); it++ )
              		ROS_ERROR_STREAM( "    "<<(*it).first);

            	return false;
			}
			
			ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
        	ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

			pub_desired_pos_ = n.advertise<std_msgs::Float64MultiArray>("desired_pos", 1000);
			pub_current_pos_ = n.advertise<std_msgs::Float64MultiArray>("current_pos", 1000);
			pub_desired_vel_ = n.advertise<std_msgs::Float64MultiArray>("desired_vel", 1000);
			pub_current_vel_ = n.advertise<std_msgs::Float64MultiArray>("current_vel", 1000);
			pub_desired_acc_ = n.advertise<std_msgs::Float64MultiArray>("desired_acc", 1000);
			pub_current_acc_ = n.advertise<std_msgs::Float64MultiArray>("current_acc", 1000);
			pub_error_ = n.advertise<std_msgs::Float64MultiArray>("error", 1000);
			pub_error_vel_ = n.advertise<std_msgs::Float64MultiArray>("error_vel", 1000);	

   			return true;
  		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			samplint_time_ = time.toSec() - last_time_;
			
			for(int i=0; i<n_joints_; i++)
			{
				desired_pos_[i] = init_pos_[i] + mag_*sin(2*PI*feq_*period.toSec()*count_);
	  			current_pos_[i] = joints_[i].getPosition();

				desired_vel_[i] = mag_*2*PI*feq_*cos(2*PI*feq_*period.toSec()*count_);
				current_vel_[i] = joints_[i].getVelocity();

				desired_acc_[i] = -mag_*2*PI*feq_*2*PI*feq_*sin(2*PI*feq_*period.toSec()*count_);
				current_acc_[i] = (current_vel_[i] - current_vel_old_[i])/period.toSec();
			
				error_[i] = desired_pos_[i] - current_pos_[i];
				error_vel_[i] = desired_vel_[i] - current_vel_[i];

				ded_[i] = desired_acc_[i] + 20.0*error_vel_[i] + 100.0*error_[i];
				tde_[i] = desired_cmd_old_[i] - Mbar_[i] * current_acc_[i];
			
		 		desired_cmd_[i] = Mbar_[i] * ded_[i] + tde_[i];

				joints_[i].setCommand(desired_cmd_[i]);

				current_vel_old_[i] = current_vel_[i];
				desired_cmd_old_[i] = desired_cmd_[i];				
			}	
			
			msg_desired_pos_.data.clear();
			msg_current_pos_.data.clear();
			msg_desired_vel_.data.clear();
			msg_current_vel_.data.clear();
			msg_desired_acc_.data.clear();
			msg_current_acc_.data.clear();
			msg_error_.data.clear();
			msg_error_vel_.data.clear();

			for(int i=0; i<JointMax; i++)
			{
				msg_desired_pos_.data.push_back(desired_pos_[i]*R2D);
				msg_current_pos_.data.push_back(current_pos_[i]*R2D);
				msg_desired_vel_.data.push_back(desired_vel_[i]*R2D);
				msg_current_vel_.data.push_back(current_vel_[i]*R2D);
				msg_desired_acc_.data.push_back(desired_acc_[i]*R2D);
				msg_current_acc_.data.push_back(current_acc_[i]*R2D);
				msg_error_.data.push_back(error_[i]*R2D);
				msg_error_vel_.data.push_back(error_vel_[i]*R2D);

			}			
			
			last_time_ = time.toSec();
			count_++;

			pub_desired_pos_.publish(msg_desired_pos_);
			pub_current_pos_.publish(msg_current_pos_);
			pub_desired_vel_.publish(msg_desired_vel_);
			pub_current_vel_.publish(msg_current_vel_);
			pub_desired_acc_.publish(msg_desired_acc_);
			pub_current_acc_.publish(msg_current_acc_);
			pub_error_.publish(msg_error_);
			pub_error_vel_.publish(msg_error_vel_);

  		}

  		void starting(const ros::Time& time)
		{
			last_time_ = 0.0;
			samplint_time_ = 0.0;
			count_ = 0.0;

			for(int i=0; i<JointMax; i++)
			{
  				init_pos_[i] = joints_[i].getPosition();

				desired_cmd_[i] = 0.0;
				desired_cmd_old_[i] = 0.0;
				desired_pos_[i] = 0.0;
				current_pos_[i] = 0.0;
				desired_vel_[i] = 0.0;
				current_vel_[i] = 0.0;
				current_vel_old_[i] = 0.0;
				desired_acc_[i] = 0.0;
				current_acc_[i] = 0.0;

				error_[i] = 0.0;
				error_vel_[i] = 0.0;
				ded_[i] = 0.0;
				tde_[i] = 0.0;
			}
			
			mag_ = 45.0*D2R;
			feq_ = 0.3;
				
			Mbar_[0] = 0.01;
			Mbar_[1] = 0.01;
			Mbar_[2] = 0.005;
			Mbar_[3] = 0.0015;
			Mbar_[4] = 0.001;
			Mbar_[5] = 0.00012;

		}

  		void stopping(const ros::Time& time) { }

	private:
		unsigned int n_joints_;
		std::vector<std::string> joint_names_;
  		std::vector<hardware_interface::JointHandle> joints_;
		std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

		KDL::Tree 	kdl_tree_;
		KDL::Chain	kdl_chain_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		KDL::JntArray G_;	// gravity vector

		double last_time_;
		double samplint_time_;
		double count_;
		
  		double init_pos_[JointMax];
		double desired_cmd_[JointMax];
		double desired_cmd_old_[JointMax];
		double desired_pos_[JointMax];
		double current_pos_[JointMax];
		double desired_vel_[JointMax];
		double current_vel_[JointMax];
		double current_vel_old_[JointMax];
		double desired_acc_[JointMax];
		double current_acc_[JointMax];
		double error_[JointMax];
		double error_vel_[JointMax];
		double ded_[JointMax];
		double tde_[JointMax];		
			
		double mag_;
		double feq_;

		double Mbar_[JointMax];

		ros::Publisher pub_desired_pos_;
		ros::Publisher pub_current_pos_;
		ros::Publisher pub_desired_vel_;
		ros::Publisher pub_current_vel_;
		ros::Publisher pub_desired_acc_;
		ros::Publisher pub_current_acc_;
		ros::Publisher pub_error_;
		ros::Publisher pub_error_vel_;

		std_msgs::Float64MultiArray msg_desired_pos_;
		std_msgs::Float64MultiArray msg_current_pos_;
		std_msgs::Float64MultiArray msg_desired_vel_;
		std_msgs::Float64MultiArray msg_current_vel_;
		std_msgs::Float64MultiArray msg_desired_acc_;
		std_msgs::Float64MultiArray msg_current_acc_;
		std_msgs::Float64MultiArray msg_error_;
		std_msgs::Float64MultiArray msg_error_vel_;
		
	};

}

PLUGINLIB_EXPORT_CLASS(elfin_controller::TimeDelayController,controller_interface::ControllerBase)

