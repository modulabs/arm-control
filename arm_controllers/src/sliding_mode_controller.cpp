#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#define SaveDataMax 8
#define JointMax 6
#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace arm_controllers{

	class SlidingModeController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~SlidingModeController() { }
		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{	
			// List of controlled joints
    		if (!n.getParam("joints", joint_names_))
			{
				ROS_ERROR("Could not find joint name");
				return false;
    		}
			n_joints_ = joint_names_.size();

			if(n_joints_ == 0)
			{
				ROS_ERROR("List of joint names is empty.");
				return false;
			}

			// urdf
			urdf::Model urdf;
			if (!urdf.initParam("robot_description"))
			{
				ROS_ERROR("Failed to parse urdf file");
            	return false;
			}

			// joint handle
			for(int i=0; i<n_joints_; i++)
			{
				try
				{
					joints_.push_back(hw->getHandle(joint_names_[i]));
				}
				catch (const hardware_interface::HardwareInterfaceException& e)
				{
					ROS_ERROR_STREAM("Exception thrown: " << e.what());
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
			if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)){
				ROS_ERROR("Failed to construct kdl tree");
				return false;
			}

			// kdl chain
			std::string root_name, tip_name;
			if (!n.getParam("root_link", root_name))
			{
				ROS_ERROR("Could not find root link name");
				return false;
    		}
			if (!n.getParam("tip_link", tip_name))
			{
				ROS_ERROR("Could not find tip link name");
				return false;
    		}
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

			gravity_ = KDL::Vector::Zero();
			gravity_(2) = -9.81;
			M_.resize(n_joints_);
			C_.resize(n_joints_);
			G_.resize(n_joints_);

			// inverse dynamics solver
			id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );

			// command and state
			tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			
			q_init_.data = Eigen::VectorXd::Zero(n_joints_);
			q_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);

			q_error_.data = Eigen::VectorXd::Zero(n_joints_);
			q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

			s_.data = Eigen::VectorXd::Zero(n_joints_);
			k_sgn_s.data = Eigen::VectorXd::Zero(n_joints_); 
			sgn_s_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_r_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_r_.data = Eigen::VectorXd::Zero(n_joints_);

			dt_ = 0.0;
			time_ = 0.0;

			// gains
			Slope_.resize(n_joints_);
			Kd_.resize(n_joints_);
			K_.resize(n_joints_);

			std::vector<double> Slope(n_joints_), Kd(n_joints_), K(n_joints_);
			for (size_t i = 0; i < n_joints_; i++)
			{
				std::string si = boost::lexical_cast<std::string>(i + 1);
				if (n.getParam("/elfin/sliding_mode_controller/joint" + si + "/smc/slope", Slope[i]))
				{
					Slope_(i) = Slope[i];
				}
				else
				{
					std::cout << "/elfin/sliding_mode_controller/joint" + si + "/smc/slope" << std::endl;
					ROS_ERROR("Cannot find smc/slope gain");
					return false;
				}

				if (n.getParam("/elfin/sliding_mode_controller/joint" + si + "/smc/kd", Kd[i]))
				{
					Kd_(i) = Kd[i];
				}
				else
				{
					std::cout << "/elfin/sliding_mode_controller/joint" + si + "/smc/kd" << std::endl;
					ROS_ERROR("Cannot find smc/kd gain");
					return false;
				}

				if (n.getParam("/elfin/sliding_mode_controller/joint" + si + "/smc/k", K[i]))
				{
					K_(i) = K[i];
				}
				else
				{
					ROS_ERROR("Cannot find smc/k gain");
					return false;
				}
			}

			pub_SaveData_Joint1_ = n.advertise<std_msgs::Float64MultiArray>("SaveData_Joint1", 1000);
			pub_SaveData_Joint2_ = n.advertise<std_msgs::Float64MultiArray>("SaveData_Joint2", 1000);
			pub_SaveData_Joint3_ = n.advertise<std_msgs::Float64MultiArray>("SaveData_Joint3", 1000);
			pub_SaveData_Joint4_ = n.advertise<std_msgs::Float64MultiArray>("SaveData_Joint4", 1000);
			pub_SaveData_Joint5_ = n.advertise<std_msgs::Float64MultiArray>("SaveData_Joint5", 1000);
			pub_SaveData_Joint6_ = n.advertise<std_msgs::Float64MultiArray>("SaveData_Joint6", 1000);

			return true;
  		}

		void starting(const ros::Time& time)
		{
			// get joint positions
			for(size_t i=0; i<n_joints_; i++) 
			{
				q_(i) = joints_[i].getPosition();
				q_init_(i) = 0.0;
				qdot_(i) = joints_[i].getVelocity();
			}

			ROS_INFO("Starting Sliding Mode Controller");
		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			dt_ = period.toSec();

			task_tunning();
			//task_init();

			id_solver_->JntToMass(q_, M_);
			id_solver_->JntToCoriolis(q_, qdot_, C_);
			id_solver_->JntToGravity(q_, G_);

			q_error_.data = q_.data - q_cmd_.data;
			q_error_dot_.data = qdot_.data - qdot_cmd_.data;

			qdot_r_.data = qdot_cmd_.data - Slope_.data.cwiseProduct(q_error_.data);
			qddot_r_.data = qddot_cmd_.data - Slope_.data.cwiseProduct(q_error_dot_.data) ;

			s_.data = qdot_.data - qdot_r_.data;

			for(int i=0; i<n_joints_; i++)
			{
				sgn_s_(i) = (-(s_(i) < 0) + (s_(i) > 0))*s_(i);
				k_sgn_s(i) = K_(i) * sgn_s_(i);
			}

			tau_cmd_.data = M_.data * qddot_r_.data + C_.data.cwiseProduct(qdot_r_.data) + G_.data - Kd_.data.cwiseProduct(s_.data) - K_.data.cwiseProduct(sgn_s_.data);

			for(int i=0; i<n_joints_; i++)
			{
				joints_[i].setCommand( tau_cmd_(i) );
			}

			data_save();

			time_ = time_ + dt_;
  		}

  		void stopping(const ros::Time& time) { }

		void task_tunning()
		{
			double mag = 45.0 * D2R;
			double freq = 0.3;

			// get joint states
			for (size_t i=0; i<n_joints_; i++)
			{
				q_cmd_(i) = q_init_(i) + mag*sin(2*PI*freq*time_);
				qdot_cmd_(i) = mag*2*PI*freq*cos(2*PI*freq*time_);
				qddot_cmd_(i) = -mag*2*PI*freq*2*PI*freq*sin(2*PI*freq*time_);

				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
			}		
		}

		void task_init()
		{
			for (size_t i=0; i<n_joints_; i++)
			{
				q_cmd_(i) = 0.0;
				qdot_cmd_(i) = 0.0;
				qddot_cmd_(i) = 0.0;
			}
		}

		void data_save()
		{
			SaveData_Joint1_[0] = time_;
			SaveData_Joint1_[1] = q_(0)*R2D;
			SaveData_Joint1_[2] = qdot_(0)*R2D;
			SaveData_Joint1_[3] = q_cmd_(0)*R2D;
			SaveData_Joint1_[4] = qdot_cmd_(0)*R2D;			
			SaveData_Joint1_[5] = q_error_(0)*R2D;
			SaveData_Joint1_[6] = q_error_dot_(0)*R2D;
			SaveData_Joint1_[7] = tau_cmd_(0);

			SaveData_Joint2_[0] = time_;
			SaveData_Joint2_[1] = q_(1)*R2D;
			SaveData_Joint2_[2] = qdot_(1)*R2D;
			SaveData_Joint2_[3] = q_cmd_(1)*R2D;
			SaveData_Joint2_[4] = qdot_cmd_(1)*R2D;			
			SaveData_Joint2_[5] = q_error_(1)*R2D;
			SaveData_Joint2_[6] = q_error_dot_(1)*R2D;
			SaveData_Joint2_[7] = tau_cmd_(1);

			SaveData_Joint3_[0] = time_;
			SaveData_Joint3_[1] = q_(2)*R2D;
			SaveData_Joint3_[2] = qdot_(2)*R2D;
			SaveData_Joint3_[3] = q_cmd_(2)*R2D;
			SaveData_Joint3_[4] = qdot_cmd_(2)*R2D;			
			SaveData_Joint3_[5] = q_error_(2)*R2D;
			SaveData_Joint3_[6] = q_error_dot_(2)*R2D;
			SaveData_Joint3_[7] = tau_cmd_(2);

			SaveData_Joint4_[0] = time_;
			SaveData_Joint4_[1] = q_(3)*R2D;
			SaveData_Joint4_[2] = qdot_(3)*R2D;
			SaveData_Joint4_[3] = q_cmd_(3)*R2D;
			SaveData_Joint4_[4] = qdot_cmd_(3)*R2D;			
			SaveData_Joint4_[5] = q_error_(3)*R2D;
			SaveData_Joint4_[6] = q_error_dot_(3)*R2D;
			SaveData_Joint4_[7] = tau_cmd_(3);

			SaveData_Joint5_[0] = time_;
			SaveData_Joint5_[1] = q_(4)*R2D;
			SaveData_Joint5_[2] = qdot_(4)*R2D;
			SaveData_Joint5_[3] = q_cmd_(4)*R2D;
			SaveData_Joint5_[4] = qdot_cmd_(4)*R2D;			
			SaveData_Joint5_[5] = q_error_(4)*R2D;
			SaveData_Joint5_[6] = q_error_dot_(4)*R2D;
			SaveData_Joint5_[7] = tau_cmd_(4);

			SaveData_Joint6_[0] = time_;
			SaveData_Joint6_[1] = q_(5)*R2D;
			SaveData_Joint6_[2] = qdot_(5)*R2D;
			SaveData_Joint6_[3] = q_cmd_(5)*R2D;
			SaveData_Joint6_[4] = qdot_cmd_(5)*R2D;			
			SaveData_Joint6_[5] = q_error_(5)*R2D;
			SaveData_Joint6_[6] = q_error_dot_(5)*R2D;
			SaveData_Joint6_[7] = tau_cmd_(5);

			msg_SaveData_Joint1_.data.clear();
			msg_SaveData_Joint2_.data.clear();
			msg_SaveData_Joint3_.data.clear();
			msg_SaveData_Joint4_.data.clear();
			msg_SaveData_Joint5_.data.clear();
			msg_SaveData_Joint6_.data.clear();

			for (size_t i = 0; i < SaveDataMax; i++)
			{
				msg_SaveData_Joint1_.data.push_back(SaveData_Joint1_[i]);
				msg_SaveData_Joint2_.data.push_back(SaveData_Joint2_[i]);
				msg_SaveData_Joint3_.data.push_back(SaveData_Joint3_[i]);
				msg_SaveData_Joint4_.data.push_back(SaveData_Joint4_[i]);
				msg_SaveData_Joint5_.data.push_back(SaveData_Joint5_[i]);
				msg_SaveData_Joint6_.data.push_back(SaveData_Joint6_[i]);
			}

			pub_SaveData_Joint1_.publish(msg_SaveData_Joint1_);
			pub_SaveData_Joint2_.publish(msg_SaveData_Joint2_);
			pub_SaveData_Joint3_.publish(msg_SaveData_Joint3_);
			pub_SaveData_Joint4_.publish(msg_SaveData_Joint4_);
			pub_SaveData_Joint5_.publish(msg_SaveData_Joint5_);
			pub_SaveData_Joint6_.publish(msg_SaveData_Joint6_);		
		}

	  private:
		// joint handles
		unsigned int n_joints_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointHandle> joints_;
		std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

		// kdl
		KDL::Tree kdl_tree_;
		KDL::Chain kdl_chain_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
		KDL::JntSpaceInertiaMatrix M_;
		KDL::JntArray C_, G_;
		KDL::Vector gravity_;

		// smc gain
		KDL::JntArray Slope_, Kd_, K_;

		// cmd, state
		KDL::JntArray q_init_;
		KDL::JntArray tau_cmd_;
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
		KDL::JntArray q_, qdot_;
		KDL::JntArray q_error_, q_error_dot_;
		KDL::JntArray s_, sgn_s_, k_sgn_s, qdot_r_, qddot_r_;

		double dt_, time_;

		// topic
		double SaveData_Joint1_[SaveDataMax];
		ros::Publisher pub_SaveData_Joint1_;
		std_msgs::Float64MultiArray msg_SaveData_Joint1_;

		double SaveData_Joint2_[SaveDataMax];
		ros::Publisher pub_SaveData_Joint2_;
		std_msgs::Float64MultiArray msg_SaveData_Joint2_;

		double SaveData_Joint3_[SaveDataMax];
		ros::Publisher pub_SaveData_Joint3_;
		std_msgs::Float64MultiArray msg_SaveData_Joint3_;

		double SaveData_Joint4_[SaveDataMax];
		ros::Publisher pub_SaveData_Joint4_;
		std_msgs::Float64MultiArray msg_SaveData_Joint4_;

		double SaveData_Joint5_[SaveDataMax];
		ros::Publisher pub_SaveData_Joint5_;
		std_msgs::Float64MultiArray msg_SaveData_Joint5_;

		double SaveData_Joint6_[SaveDataMax];
		ros::Publisher pub_SaveData_Joint6_;
		std_msgs::Float64MultiArray msg_SaveData_Joint6_;												
	};
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::SlidingModeController, controller_interface::ControllerBase)

