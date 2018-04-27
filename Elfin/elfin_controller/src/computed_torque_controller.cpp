#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include <boost/lexical_cast.hpp>

namespace elfin_controller{

	class ComputedTorqueController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{	
			// joint name
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
			std::string root_name = "world";
			std::string tip_name = "elfin_link6";
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
			// ver. 01 gdyn ver
			M_.resize(n_joints_);
			C_.resize(n_joints_);
			G_.resize(n_joints_);	

			// ver. 02 kuka
			//M_.resize(kdl_chain_.getNrOfJoints());
			//C_.resize(kdl_chain_.getNrOfJoints());
			//G_.resize(kdl_chain_.getNrOfJoints());
			
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

			e_.data = Eigen::VectorXd::Zero(n_joints_);
			e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
			e_int_.data = Eigen::VectorXd::Zero(n_joints_);

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
				if ( n.getParam("/elfin/computed_torque_controller/joint" + si + "/pid/p", Kp[i]) )
				{
					Kp_(i) = Kp[i];
				}
				else
				{
					std::cout << "/elfin/computed_torque_controller/joint" + si + "/pid/p" << std::endl;
					ROS_ERROR("Cannot find pid/p gain");
					return false;
				}

				if ( n.getParam("/elfin/computed_torque_controller/joint" + si + "/pid/i", Ki[i]) )
				{
					Ki_(i) = Ki[i];
				}
				else
				{
					ROS_ERROR("Cannot find pid/i gain");
					return false;
				}

				if ( n.getParam("/elfin/computed_torque_controller/joint" + si + "/pid/d", Kd[i]) )
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

			// subscribe command
			sub_q_cmd_ = n.subscribe("command", 1, &ComputedTorqueController::setCommandCB, this);

			ROS_INFO("End of computed torque controller init");
			// ser
			//ros::ServiceServer srv_load_gain_ = n.advertiseService("load_gain", &ComputedTorqueController::loadGainCB, this);

			// 7. ROS 명령어?
			pub_q_cmd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
			pub_qdot_cmd_ = n.advertise<std_msgs::Float64MultiArray>("qd_dot", 1000);
			pub_qddot_cmd_ = n.advertise<std_msgs::Float64MultiArray>("qd_ddot", 1000);

			pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);		// 뒤에 숫자는?
			pub_qdot_ = n.advertise<std_msgs::Float64MultiArray>("qdot", 1000); // 뒤에 숫자는?

			pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);
			pub_e_dot_ = n.advertise<std_msgs::Float64MultiArray>("e_dot", 1000);
			pub_e_int_ = n.advertise<std_msgs::Float64MultiArray>("e_int", 1000);

			return true;
  		}

		void starting(const ros::Time& time)
		{
			ROS_INFO("Start Computed Torque Controller controller");
			// get joint positions
			for(size_t i=0; i<n_joints_; i++) 
			{
				ROS_INFO("JOINT %d", (int)i);
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
				q_error_integral_(i) = .0;
			}

			ROS_INFO("Starting Computed Torque Control");
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
			// simple trajectory interpolation from joint command setpoint (??)
			double dt = period.toSec();
			double qdot_cmd;
			double qddot_cmd; // jungyeong
			
			static double t=.0;
			for (size_t i=0; i<n_joints_; i++)
			{
				
				// double dq = q_cmd_sp_(i) - q_cmd_(i);	// [to do] shortest distance using angle
				// double qdot_cmd = KDL::sign(dq) * KDL::min( fabs(dq/dt), fabs(qdot_limit_(i)) );
				// double ddq = qdot_cmd - qdot_cmd_(i);	// [to do] shortest distance using angle
				// double qddot_cmd = KDL::sign(ddq) * KDL::min( fabs(ddq/dt), fabs(qddot_limit_(i)) ); //jungyeong
				
				// qdot_cmd_(i) += qddot_cmd*dt;
				// q_cmd_(i) += qdot_cmd*dt;
				qddot_cmd_(i)= -M_PI*M_PI*30*KDL::deg2rad*sin(M_PI*t); // jungyeong
				qdot_cmd_(i)= M_PI*30*KDL::deg2rad*cos(M_PI*t); // jungyeong
				q_cmd_(i) = 30*KDL::deg2rad*sin(M_PI*t);
			}
			t = t + 0.001;

			// get joint states
			for (size_t i=0; i<n_joints_; i++)
			{
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
			}
			
			// compute M,C,G Torque
			id_solver_->JntToMass(q_, M_);
			id_solver_->JntToCoriolis(q_,qdot_, C_);
			id_solver_->JntToGravity(q_, G_);

			// error
			

			e_dot_.data = qdot_cmd_.data - qdot_.data;
			e_.data = q_cmd_.data - q_.data;	// [to do] shortest distance using angle
			e_int_.data += e_.data;
			
			// integral saturation
			for (size_t i; i<n_joints_; i++)
			{
				if (q_error_integral_(i) > q_integral_max_)
					q_error_integral_(i) = q_integral_max_;
				else if (q_error_integral_(i) < -q_integral_max_)
					q_error_integral_(i) = -q_integral_max_;
			}

			// torque command
			aux_cmd_.data = M_.data*( qddot_cmd_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data) );
			comp_cmd_.data = C_.data + G_.data;
			tau_cmd_.data = aux_cmd_.data + comp_cmd_.data;
			


			for(int i=0; i<n_joints_; i++)
			{
				joints_[i].setCommand(tau_cmd_(i));
			}

			// 3. ROS (Data save, 3.1~3.3)

			// 3.1
			msg_q_cmd_.data.clear();
			msg_qdot_cmd_.data.clear();
			msg_qddot_cmd_.data.clear();

			msg_q_.data.clear();
			msg_qdot_.data.clear();

			msg_e_.data.clear();
			msg_e_dot_.data.clear();
			msg_e_int_.data.clear();


			// 3.2
			for (int i = 0; i < n_joints_; i++)
			{
				msg_q_cmd_.data.push_back(q_cmd_(i));
				msg_qdot_cmd_.data.push_back(qdot_cmd_(i));
				msg_qddot_cmd_.data.push_back(qddot_cmd_(i));

				msg_q_.data.push_back(q_(i));
				msg_qdot_.data.push_back(qdot_(i));

				msg_e_.data.push_back(e_(i));
				msg_e_dot_.data.push_back(e_dot_(i));
				msg_e_int_.data.push_back(e_int_(i));

			}

			// 3.3
			pub_q_cmd_.publish(msg_q_cmd_);
			pub_qdot_cmd_.publish(msg_qdot_cmd_);
			pub_qddot_cmd_.publish(msg_qddot_cmd_);

			pub_q_.publish(msg_q_);
			pub_qdot_.publish(msg_qdot_);

			pub_e_.publish(msg_e_);
			pub_e_dot_.publish(msg_e_dot_);
			pub_e_int_.publish(msg_e_int_);
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
		KDL::JntSpaceInertiaMatrix M_;						// Mass and Inertia matirx?
		KDL::JntArray C_;									// Coriolis matrix?
		KDL::JntArray G_;									// gravity torque vector
		KDL::Vector gravity_;

		// pid gain
		KDL::JntArray Kp_, Ki_, Kd_;						// p,i,d gain
		double q_integral_min_;
		double q_integral_max_;
		KDL::JntArray q_error_integral_;

		// cmd, state
		KDL::JntArray tau_cmd_;
		KDL::JntArray aux_cmd_;
		KDL::JntArray comp_cmd_;
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
		KDL::JntArray q_cmd_sp_;
		KDL::JntArray q_, qdot_;
		KDL::JntArray e_, e_dot_,e_int_;


		// limit
		KDL::JntArray q_limit_min_, q_limit_max_, qdot_limit_, qddot_limit_;

		// topic
		ros::Subscriber sub_q_cmd_;

		// ros publisher ?
		ros::Publisher pub_q_cmd_, pub_qdot_cmd_, pub_qddot_cmd_;
		ros::Publisher pub_q_, pub_qdot_;
		ros::Publisher pub_e_, pub_e_dot_, pub_e_int_;

		ros::Publisher pub_M_, pub_C_, pub_G_;

		// ros message ?
		std_msgs::Float64MultiArray msg_q_cmd_, msg_qdot_cmd_, msg_qddot_cmd_;
		std_msgs::Float64MultiArray msg_q_, msg_qdot_;
		std_msgs::Float64MultiArray msg_e_, msg_e_dot_, msg_e_int_;

		std_msgs::Float64MultiArray msg_M_, msg_C_, msg_G_;
	};
}

PLUGINLIB_EXPORT_CLASS(elfin_controller::ComputedTorqueController, controller_interface::ControllerBase)

