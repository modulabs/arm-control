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

namespace arm_controllers{

	class PassivityController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~PassivityController() {sub_command_.shutdown();}
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
			qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			
			q_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);

			q_error_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_error_.data = Eigen::VectorXd::Zero(n_joints_);

			// gains
			alpha_.resize(n_joints_);
			pid_controllers_.resize(n_joints_);
			for (size_t i=0; i<n_joints_; i++)
			{
				if (!n.getParam("gains/" + joint_names_[i] + "/alpha", alpha_[i]))
				{
					ROS_ERROR("Could not find root link name");
					return false;
				}

				// Load PID Controller using gains set on parameter server
				if (!pid_controllers_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
				{
					ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
					return false;
				}
			}

			// command
			commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
			sub_command_ = n.subscribe("command", 1, &PassivityController::commandCB, this);

			// ser
			//ros::ServiceServer srv_load_gain_ = n.advertiseService("load_gain", &PassivityController::loadGainCB, this);

   			return true;
  		}

		void starting(const ros::Time& time)
		{
			// get joint positions
			for(size_t i=0; i<n_joints_; i++) 
			{
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
			}

			ROS_INFO("Starting Passivity Based Controller");
		}

		void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
		{
			if(msg->data.size()!=n_joints_)
			{ 
			ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
			return; 
			}
			commands_buffer_.writeFromNonRT(msg->data);
		}

		// load gain is not permitted during controller loading?
		void loadGainCB()
		{
			
		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			std::vector<double> & commands = *commands_buffer_.readFromRT();
			
			// get joint states
			static double t = 0;
			for (size_t i=0; i<n_joints_; i++)
			{
				q_cmd_(i) = M_PI/6*sin(t/10);//commands[i];
				enforceJointLimits(q_cmd_(i), i);
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();

		        // Compute position error
				if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
				{
					angles::shortest_angular_distance_with_limits(
						q_(i),
						q_cmd_(i),
						joint_urdfs_[i]->limits->lower,
						joint_urdfs_[i]->limits->upper,
						q_error_(i));
				}
				else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
				{
					q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
				}
				else //prismatic
				{
					q_error_(i) = q_cmd_(i) - q_(i);
				}
			}

			qdot_cmd_.data = (q_cmd_.data - q_cmd_old_.data) / period.toSec();;
			qddot_cmd_.data = (qdot_cmd_.data - qdot_cmd_old_.data) / period.toSec();
			
			qdot_error_.data = qdot_cmd_.data - qdot_.data;

			q_cmd_old_ = q_cmd_;
			qdot_cmd_old_ = qdot_cmd_;

			// double dt = period.toSec();
			// if (t==10)
			// {
			// 	ROS_INFO("q_cmd = %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
			// 		q_cmd_(0), q_cmd_(1), q_cmd_(2), q_cmd_(3), q_cmd_(4), q_cmd_(5));
			// 	t = 0;
			// }
			t++;
			
			// compute gravity torque
			id_solver_->JntToMass(q_, M_);
			id_solver_->JntToCoriolis(q_, C_);
			id_solver_->JntToGravity(q_, G_);

			// torque command
			qdot_ref_ = qdot_cmd_.data + alpha_.data.cwiseProduct(q_error_.data);
			qddot_ref_ = qddot_cmd.data + alpha_.data.cwiseProduct(qdot_error_.data);

			tau_cmd_.data = M_.data * qddot_ref_.data + C_.data.cwiseProduct(qdot_ref_.data) + G_.data;

			for(int i=0; i<n_joints_; i++)
			{
				tau_cmd_(i) += pid_controllers_[i].computeCommand(q_error_(i), qdot_error_(i), period.toSec());
				joints_[i].setCommand( tau_cmd_(i) );
			}
  		}

  		void stopping(const ros::Time& time) { }

		void enforceJointLimits(double &command, unsigned int index)
		{
			// Check that this joint has applicable limits
			if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
			{
			if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
			{
				command = joint_urdfs_[index]->limits->upper;
			}
			else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
			{
				command = joint_urdfs_[index]->limits->lower;
			}
			}
		}
	private:
		// joint handles
		unsigned int n_joints_;
		std::vector<std::string> joint_names_;
  		std::vector<hardware_interface::JointHandle> joints_;
		std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

		// kdl
		KDL::Tree 	kdl_tree_;
		KDL::Chain	kdl_chain_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	// inverse dynamics solver
		KDL::JntSpaceInertiaMatrix M_;
		KDL::JntArray C_;
		KDL::JntArray G_;									// gravity torque vector
		KDL::Vector gravity_;

		// pid gain
		KDL::JntArray alpha_;								// error weight in passivity-based control joint reference
  		std::vector<control_toolbox::Pid> pid_controllers_; // Internal PID controllers

		// cmd, state
		realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
		KDL::JntArray tau_cmd_;
		KDL::JntArray qdot_ref_, qddot_ref_;				// passivity-based control joint reference
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
		KDL::JntArray q_, qdot_;
		KDL::JntArray q_error_, qdot_error_;

		// topic
		ros::Subscriber sub_command_;
	};

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::PassivityController, controller_interface::ControllerBase)

