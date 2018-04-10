#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

namespace arm_controllers{

	class GravityCompController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
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
			else
			{
				ROS_INFO("Found %d joint names", n_joints_);
			}

			// urdf
			urdf::Model urdf;
			if (!urdf.initParam("robot_description"))
			{
				ROS_ERROR("Failed to parse urdf file");
            	return false;
			}
			else
			{
				ROS_INFO("Found robot_description");
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
			else
			{
				ROS_INFO("Constructed kdl tree");
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
			else
			{
				ROS_INFO("Got kdl chain");
			}
			
			ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
        	ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

			gravity_ = KDL::Vector::Zero();
			gravity_(2) = -9.81;
			G_.resize(n_joints_);	
			
			// inverse dynamics solver
			id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );

			// command and state
			tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_sp_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			
			q_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);

			// limit [to do] read from parameter
			q_limit_min_.resize(n_joints_); q_limit_max_.resize(n_joints_);
			q_limit_min_.data << -90*KDL::deg2rad, -90*KDL::deg2rad, -90*KDL::deg2rad, -90*KDL::deg2rad, -90*KDL::deg2rad, -90*KDL::deg2rad;
			q_limit_max_.data << 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad; 
			qdot_limit_.resize(n_joints_);	
			qdot_limit_.data << 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad;
			qddot_limit_.resize(n_joints_);
			qddot_limit_.data << 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad, 90*KDL::deg2rad;
			

			// pid gains
			for (size_t i=0; i<n_joints_; i++)
			{
				// Load PID Controller using gains set on parameter server
				if (!pid_controllers_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
				{
					ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
					return false;
				}
			}

			// subscribe command
			sub_q_cmd_ = n.subscribe("command", 1, &GravityCompController::setCommandCB, this);

			ROS_INFO("End of gravity controller init");
			// ser
			//ros::ServiceServer srv_load_gain_ = n.advertiseService("load_gain", &GravityCompController::loadGainCB, this);

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

		void setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
		{
			if(msg->data.size()!=n_joints_)
			{ 
				ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
				return; 
			}

    		for (unsigned int i = 0; i<n_joints_; i++)
    			q_cmd_sp_(i) = msg->data[i];
		}

		// load gain is not permitted during controller loading?
		void loadGainCB()
		{
			
		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			// simple trajectory interpolation from joint command setpoint
			double dt = period.toSec();
			double qdot_cmd;
			
			static double t=.0;
			for (size_t i=0; i<n_joints_; i++)
			{
				double dq = q_cmd_sp_(i) - q_cmd_(i);	// [to do] shortest distance using angle
				double qdot_cmd = KDL::sign(dq) * KDL::min( fabs(dq/dt), fabs(qdot_limit_(i)) );
				double ddq = qdot_cmd - qdot_cmd_(i);	// [to do] shortest distance using angle
				double qddot_cmd = KDL::sign(ddq) * KDL::min( fabs(ddq/dt), fabs(qddot_limit_(i)) );
				qdot_cmd_(i) += qddot_cmd*dt;
				q_cmd_(i) += qdot_cmd*dt;
				q_cmd_(i) = 30*KDL::deg2rad*sin(M_PI*t);
			}
			t = t + dt;

			// get joint states
			for (size_t i=0; i<n_joints_; i++)
			{
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
			}
			
			// compute gravity torque
			id_solver_->JntToGravity(q_, G_);

			// error
			KDL::JntArray q_error;
			q_error.data = q_cmd_.data - q_.data;	// [to do] shortest distance using angle
			
			// torque command
			for(int i=0; i<n_joints_; i++)
			{
				tau_cmd_(i) = G_(i) + pid_controllers_[i].computeCommand(q_error(i), period);
				joints_[i].setCommand( tau_cmd_(i) );
			}
  		}

  		void stopping(const ros::Time& time) { }

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
		KDL::JntArray G_;									// gravity torque vector
		KDL::Vector gravity_;

		// pid gain
  		std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

		// cmd, state
		KDL::JntArray tau_cmd_;
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
		KDL::JntArray q_cmd_sp_;
		KDL::JntArray q_, qdot_;

		// limit
		KDL::JntArray q_limit_min_, q_limit_max_, qdot_limit_, qddot_limit_;

		// topic
		ros::Subscriber sub_q_cmd_;
	};

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityCompController, controller_interface::ControllerBase)

