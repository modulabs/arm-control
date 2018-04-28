#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
#define JointMax 6
#define SaveDataMax 8

namespace arm_controllers{

	class AdaptiveImpedanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~AdaptiveImpedanceController() {sub_q_cmd_.shutdown(); sub_forcetorque_sensor_.shutdown();}

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
			if (!urdf.initParam("elfin/robot_description"))
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
			
			// gains
			Kp_.resize(n_joints_);
			Kd_.resize(n_joints_);
			Ki_.resize(n_joints_);
			q_error_integral_.resize(n_joints_);
			q_integral_min_ = .0;
			q_integral_max_ = .0;	// [to do] assign proper value

			std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
			for (size_t i=0; i<n_joints_; i++)
			{
				std::string si = boost::lexical_cast<std::string>(i+1);
				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/pid/p", Kp[i]) )
				{
					Kp_(i) = Kp[i];
				}
				else
				{
					std::cout << "/elfin/adaptive_impedance_controller/joint" + si + "/pid/p" << std::endl;
					ROS_ERROR("Cannot find pid/p gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/pid/i", Ki[i]) )
				{
					Ki_(i) = Ki[i];
				}
				else
				{
					ROS_ERROR("Cannot find pid/i gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/pid/d", Kd[i]) )
				{
					Kd_(i) = Kd[i];
				}
				else
				{
					ROS_ERROR("Cannot find pid/d gain");
					return false;
				}
				
				q_error_integral_(i) = .0;
			}

			// command
			sub_q_cmd_ = n.subscribe("command", 1, &AdaptiveImpedanceController::commandCB, this);
			sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/elfin/elfin/ft_sensor_topic", 1, &AdaptiveImpedanceController::updateFTsensor, this);

			pub_Joint1_Data_ = n.advertise<std_msgs::Float64MultiArray>("Joint1_Data", 1000);
			pub_Joint2_Data_ = n.advertise<std_msgs::Float64MultiArray>("Joint2_Data", 1000);
			pub_Joint3_Data_ = n.advertise<std_msgs::Float64MultiArray>("Joint3_Data", 1000);
			pub_Joint4_Data_ = n.advertise<std_msgs::Float64MultiArray>("Joint4_Data", 1000);
			pub_Joint5_Data_ = n.advertise<std_msgs::Float64MultiArray>("Joint5_Data", 1000);
			pub_Joint6_Data_ = n.advertise<std_msgs::Float64MultiArray>("Joint6_Data", 1000);

   			return true;
  		}

		void starting(const ros::Time& time)
		{
			// get joint positions
			for(size_t i=0; i<n_joints_; i++) 
			{
				ROS_INFO("JOINT %d", (int)i);
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
				q_error_integral_(i) = .0;
			}

			count_ = 0.0;

			ROS_INFO("Starting Adaptive Impedance Controller");
		}

		void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
		{
			if(msg->data.size()!=n_joints_)
			{ 
				ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
				return; 
			}

    		for (unsigned int i = 0; i<n_joints_; i++)
    			q_cmd_sp_(i) = msg->data[i];
		}

		//void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
		void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
		{
			// Convert Wrench msg to KDL wrench
			geometry_msgs::Wrench f_meas = msg->wrench;

			f_cur_[0] = f_meas.force.x;
			f_cur_[1] = f_meas.force.y;
			f_cur_[2] = f_meas.force.z;
			f_cur_[3] = f_meas.torque.x;
			f_cur_[4] = f_meas.torque.y;
			f_cur_[5] = f_meas.torque.z;
		}

		// load gain is not permitted during controller loading?
		void loadGainCB()
		{
			
		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			//ROS_INFO("force_x = %.5f, force_y = %.5f, force_z = %.5f\n", f_cur_[0], f_cur_[1], f_cur_[2]);
			//ROS_INFO("torque_x = %.5f, torque_y = %.5f, torque_z = %.5f\n", f_cur_[3], f_cur_[4], f_cur_[5]);

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
				//q_cmd_(i) = 0;
			}
			t = t + 0.001;

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
			KDL::JntArray q_error_dot;

			q_error.data = q_cmd_.data - q_.data;
			q_error_dot.data = qdot_cmd_.data - qdot_.data;
			q_error_integral_.data += q_error.data;
			
			// integral saturation
			for (size_t i; i<n_joints_; i++)
			{
				if (q_error_integral_(i) > q_integral_max_)
					q_error_integral_(i) = q_integral_max_;
				else if (q_error_integral_(i) < -q_integral_max_)
					q_error_integral_(i) = -q_integral_max_;
			}

			// torque command
			tau_cmd_.data = G_.data + Kp_.data.cwiseProduct(q_error.data) + Kd_.data.cwiseProduct(q_error_dot.data) + Ki_.data.cwiseProduct(q_error_integral_.data);

			for(int i=0; i<n_joints_; i++)
			{
				joints_[i].setCommand(tau_cmd_(i));
			}

			for(int i=0; i<JointMax; i++)
			{
				JointData_[i][0] = count_;
				JointData_[i][1] = q_cmd_(i)*R2D;
				JointData_[i][2] = q_(i)*R2D;	
				JointData_[i][3] = qdot_cmd_(i)*R2D;
				JointData_[i][4] = qdot_(i)*R2D;
				JointData_[i][5] = tau_cmd_(i);
				JointData_[i][6] = q_error(i)*R2D;
				JointData_[i][7] = q_error_dot(i)*R2D;
			}
				
			msg_Joint1_Data_.data.clear();
			msg_Joint2_Data_.data.clear();
			msg_Joint3_Data_.data.clear();
			msg_Joint4_Data_.data.clear();
			msg_Joint5_Data_.data.clear();
			msg_Joint6_Data_.data.clear();

			for(int i=0; i<SaveDataMax; i++)
			{
				msg_Joint1_Data_.data.push_back(JointData_[0][i]);
				msg_Joint2_Data_.data.push_back(JointData_[1][i]);
				msg_Joint3_Data_.data.push_back(JointData_[2][i]);
				msg_Joint4_Data_.data.push_back(JointData_[3][i]);
				msg_Joint5_Data_.data.push_back(JointData_[4][i]);
				msg_Joint6_Data_.data.push_back(JointData_[5][i]);
			}						

			count_++;

			pub_Joint1_Data_.publish(msg_Joint1_Data_);
			pub_Joint2_Data_.publish(msg_Joint2_Data_);
			pub_Joint3_Data_.publish(msg_Joint3_Data_);
			pub_Joint4_Data_.publish(msg_Joint4_Data_);
			pub_Joint5_Data_.publish(msg_Joint5_Data_);
			pub_Joint6_Data_.publish(msg_Joint6_Data_);						
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
		KDL::JntArray Kp_, Ki_, Kd_;						// p,i,d gain
		KDL::JntArray q_error_integral_;
		double q_integral_min_;
		double q_integral_max_;      /**< Internal PID controllers. */

		// cmd, state
		KDL::JntArray tau_cmd_;
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
		KDL::JntArray q_cmd_sp_;
		KDL::JntArray q_, qdot_;
		KDL::Wrench f_cur_;

		// limit
		KDL::JntArray q_limit_min_, q_limit_max_, qdot_limit_, qddot_limit_;

		// topic
		ros::Subscriber sub_q_cmd_;
		ros::Subscriber sub_forcetorque_sensor_;

		double count_;
		double JointData_[JointMax][SaveDataMax];

		ros::Publisher pub_Joint1_Data_;
		ros::Publisher pub_Joint2_Data_;
		ros::Publisher pub_Joint3_Data_;
		ros::Publisher pub_Joint4_Data_;
		ros::Publisher pub_Joint5_Data_;
		ros::Publisher pub_Joint6_Data_;

		std_msgs::Float64MultiArray msg_Joint1_Data_;
		std_msgs::Float64MultiArray msg_Joint2_Data_;
		std_msgs::Float64MultiArray msg_Joint3_Data_;
		std_msgs::Float64MultiArray msg_Joint4_Data_;
		std_msgs::Float64MultiArray msg_Joint5_Data_;
		std_msgs::Float64MultiArray msg_Joint6_Data_;		
	};

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdaptiveImpedanceController, controller_interface::ControllerBase)

