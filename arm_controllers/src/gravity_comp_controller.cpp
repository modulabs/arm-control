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
#include <boost/lexical_cast.hpp>

#include "arm_controllers/ControllerJointState.h"
#include "arm_controllers/GravityCompControllerParamsConfig.h"

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace arm_controllers{

	class GravityCompController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		class Gains
		{
		public:
			Gains()
				: p_(0.0), i_(0.0), d_(0.0)
			{}

			void setGains(double p, double i, double d)
			{
				p_ = p; i_ = i; d_ = d;
			}

			void getGains(double& p, double& i, double& d)
			{
				p = p_; i = i_; d = d_;
			}

			void initDynamicReconfig(ros::NodeHandle &node)
			{
				ROS_DEBUG_STREAM_NAMED("pid","Initializing dynamic reconfigure in namespace "
					<< node.getNamespace());

				// Start dynamic reconfigure server
				param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
				dynamic_reconfig_initialized_ = true;

				// Set Dynamic Reconfigure's gains to Pid's values
				updateDynamicReconfig();

				// Set callback
				param_reconfig_callback_ = boost::bind(&Gains::dynamicReconfigCallback, this, _1, _2);
				param_reconfig_server_->setCallback(param_reconfig_callback_);
			}

			void updateDynamicReconfig()
			{
				// Make sure dynamic reconfigure is initialized
				if(!dynamic_reconfig_initialized_)
					return;

				// Get starting values
				arm_controllers::GravityCompControllerParamsConfig config;

				// Get starting values
				getGains(config.p, config.i, config.d);

				updateDynamicReconfig(config);
			}

			void updateDynamicReconfig(GravityCompControllerParamsConfig config)
			{
				// Make sure dynamic reconfigure is initialized
				if(!dynamic_reconfig_initialized_)
					return;

				// Set starting values, using a shared mutex with dynamic reconfig
				param_reconfig_mutex_.lock();
				param_reconfig_server_->updateConfig(config);
				param_reconfig_mutex_.unlock();
			}

			void dynamicReconfigCallback(arm_controllers::GravityCompControllerParamsConfig &config, uint32_t /*level*/)
			{
				ROS_DEBUG_STREAM_NAMED("pid","Dynamics reconfigure callback recieved.");

				// Set the gains
				setGains(config.p, config.i, config.d);
			}

			double p_;
			double i_;
			double d_;

		private:
			// Store the PID gains in a realtime buffer to allow dynamic reconfigure to update it without
			// blocking the realtime update loop
			// realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

			// Dynamics reconfigure
			bool dynamic_reconfig_initialized_;
			typedef dynamic_reconfigure::Server<arm_controllers::GravityCompControllerParamsConfig> DynamicReconfigServer;
			boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
			DynamicReconfigServer::CallbackType param_reconfig_callback_;

			boost::recursive_mutex param_reconfig_mutex_;
		};

		public:
		~GravityCompController() 
		{
			command_sub_.shutdown();
			// for (int i=0; i<n_joints_; i++)
			// {
			// 	if (gains_[i] != NULL)
			// 	{
			// 		delete gains_[i];
			// 		gains_[i] = NULL;
			// 	}	
			// }
		}

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{	
			loop_count_ = 0;
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
			G_.resize(n_joints_);	
			
			// inverse dynamics solver
			id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );

			// command and state
			tau_cmd_ = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			
			q_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);

			q_error_ = Eigen::VectorXd::Zero(n_joints_);
			q_error_dot_ = Eigen::VectorXd::Zero(n_joints_);
			q_error_int_ = Eigen::VectorXd::Zero(n_joints_);

			ROS_INFO("helloooooooooooooooooooooooooooooooooooooooooo");
			// gains
			// kp_ = Eigen::VectorXd::Zero(n_joints_);
			// ki_ = Eigen::VectorXd::Zero(n_joints_);
			// kd_ = Eigen::VectorXd::Zero(n_joints_);

			// i_clamp_min_ = Eigen::VectorXd::Zero(n_joints_);
			// i_clamp_max_ = Eigen::VectorXd::Zero(n_joints_);

			// pid_controllers_.resize(n_joints_);
			// gains_.resize(n_joints_);
			std::string gain;
			for (size_t i=0; i<n_joints_; i++)
			{
				// if (!n.getParam("gains/" + joint_names_[i] + "/pid/p", gain))
				// {
				// 	std::cout << "gains/" + joint_names_[i] + "/pid/p" << std::endl;
				// 	ROS_ERROR("Could not find p gain %s");
				// 	return false;
				// }
				// gains_[i].p_ = boost::lexical_cast<double>(gain);

				// ROS_INFO("P = %f", gains_[i].p_);
				// if (!n.getParam("gains/" + joint_names_[i] + "/pid/i", gain))
				// {
				// 	ROS_ERROR("Could not find i gain");
				// 	return false;
				// }
				// gains_[i].i_ = boost::lexical_cast<double>(gain);

				// if (!n.getParam("gains/" + joint_names_[i] + "/pid/d", gain))
				// {
				// 	ROS_ERROR("Could not find d gain");
				// 	return false;
				// }
				// gains_[i].d_ = boost::lexical_cast<double>(gain);

				// gains_[i].initDynamicReconfig(ros::NodeHandle(n, "gains/"));
				gains_[i].initDynamicReconfig(n);
				// if (!n.getParam(joint_names_[i] + "/pid/i_clamp", gain))
				// {
				// 	ROS_ERROR("Could not find i clamp");
				// 	return false;
				// }
				// i_clamp_max_[i] = boost::lexical_cast<double>(gain);

				// // Load PID Controller using gains set on parameter server
				// if (!pid_controllers_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
				// {
				// 	ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
				// 	return false;
				// }
			}
			

			// command subscriber
			commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
			command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &GravityCompController::commandCB, this);

			// Start realtime state publisher
			controller_state_pub_.reset(
				new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

			controller_state_pub_->msg_.header.stamp = ros::Time::now();
			for (size_t i=0; i<n_joints_; i++)
			{
				controller_state_pub_->msg_.name.push_back(joint_names_[i]);
				controller_state_pub_->msg_.command.push_back(0.0);
				controller_state_pub_->msg_.command_dot.push_back(0.0);
				controller_state_pub_->msg_.state.push_back(0.0);
				controller_state_pub_->msg_.state_dot.push_back(0.0);
				controller_state_pub_->msg_.error.push_back(0.0);
				controller_state_pub_->msg_.error_dot.push_back(0.0);
				controller_state_pub_->msg_.effort_command.push_back(0.0);
				controller_state_pub_->msg_.effort_gravity.push_back(0.0);
				controller_state_pub_->msg_.effort_feedback.push_back(0.0);
			}
			
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

			ROS_INFO("Starting Gravity Compensation Controller");
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
			double dt = period.toSec();
			double q_cmd_old;

			static double t = 0;
			// get joint states
			for (size_t i=0; i<n_joints_; i++)
			{
				q_cmd_old = q_cmd_(i);
				
				if (i==3)
				{
					q_cmd_(i) = 45*D2R*sin(PI/2*t);
				}
				// else if (i==4)
				// {
				// 	q_cmd_(4) = 90*D2R;
				// }
				else
					q_cmd_(i) = commands[i];

				// q_cmd_(2) = 90*D2R;
				enforceJointLimits(q_cmd_(i), i);
				qdot_cmd_(i) = ( q_cmd_(i) - q_cmd_old )/dt;

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
				else // prismatic
				{
					q_error_(i) = q_cmd_(i) - q_(i);
				}
				q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);
			}
			t += dt;
			
			id_solver_->JntToGravity(q_, G_);

			static int td = 0;
			// torque command
			for(int i=0; i<n_joints_; i++)
			{
				// // i clamp
				// q_error_int_(i) += q_error_(i);
				// if (q_error_int_(i) >= i_clamp_max_(i))
				// 	q_error_int_(i) = i_clamp_max_(i);
				// else if (q_error_int_(i) <= i_clamp_min_(i))
				// 	q_error_int_(i) = i_clamp_min_(i);

				// 
				tau_cmd_(i) = G_(i) + gains_[i].p_*q_error_(i) + gains_[i].i_*q_error_int_(i) + gains_[i].d_*q_error_dot_(i);

				// effort saturation
				if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
					tau_cmd_(i) = joint_urdfs_[i]->limits->effort;
				
				if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
					tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

				joints_[i].setCommand( tau_cmd_(i) );
			}
			td++;

			// publish
			if (loop_count_ % 10 == 0)
			{
				if (controller_state_pub_->trylock())
				{
					controller_state_pub_->msg_.header.stamp = time;
					for(int i=0; i<n_joints_; i++)
					{
						controller_state_pub_->msg_.command[i] = R2D*q_cmd_(i);
						controller_state_pub_->msg_.command_dot[i] = R2D*qdot_cmd_(i);
						controller_state_pub_->msg_.state[i] = R2D*q_(i);
						controller_state_pub_->msg_.state_dot[i] = R2D*qdot_(i);
						controller_state_pub_->msg_.error[i] = R2D*q_error_(i);
						controller_state_pub_->msg_.error_dot[i] = R2D*q_error_dot_(i);
						controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
						controller_state_pub_->msg_.effort_gravity[i] = G_(i);
						controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - G_(i);
					}
					controller_state_pub_->unlockAndPublish();
				}
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
		int loop_count_;

		// joint handles
		unsigned int n_joints_;
		std::vector<std::string> joint_names_;
  		std::vector<hardware_interface::JointHandle> joints_;
		std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

		// kdl
		KDL::Tree 	kdl_tree_;
		KDL::Chain	kdl_chain_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	// inverse dynamics solver
		KDL::JntArray G_;									// gravity torque vector
		KDL::Vector gravity_;

		// pid gain
  		// std::vector<control_toolbox::Pid> pid_controllers_; /**< Internal PID controllers. */

		// cmd, state
		realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
		KDL::JntArray q_, qdot_;

		Eigen::VectorXd tau_cmd_;
		Eigen::VectorXd q_error_, q_error_dot_, q_error_int_;

		// gain
		// std::vector<Gains*> gains_;
		Gains gains_[6];

		// Eigen::VectorXd kp_, ki_, kd_;
		// Eigen::VectorXd i_clamp_max_, i_clamp_min_;
		// std::vector<bool> antiwindup_;

		// topic
		ros::Subscriber command_sub_;
		boost::scoped_ptr<
			realtime_tools::RealtimePublisher<
				arm_controllers::ControllerJointState> > controller_state_pub_;
		
		// // Dynamic reconfigure
		// bool dynamic_reconfig_initialized_;
		// typedef dynamic_reconfigure::Server<gravity_comp_controller::GravityCompControllerParamsConfig> DynamicReconfigServer;
		// boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
		// DynamicReconfigServer::CallbackType param_reconfig_callback_;

		// boost::recursive_mutex param_reconfig_mutex_;
	};

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityCompController, controller_interface::ControllerBase)

