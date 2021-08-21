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
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
#define JointMax 6
#define SaveDataMax 7

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
			fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
			ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
			//ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, fk_solver_, ik_vel_solver_));

			// command and state
			tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			tau_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_sp_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_end_.data = Eigen::VectorXd::Zero(n_joints_);
			
			q_.data = Eigen::VectorXd::Zero(n_joints_);
			q_init_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_.data = Eigen::VectorXd::Zero(n_joints_);

			for (size_t i = 0; i < 6; i++)
			{
				Xc_dot_(i) = 0.0;
				Xc_dot_old_(i) = 0.0;
				Xc_ddot_(i) = 0.0;
			}

			// gains
			Mbar_.resize(n_joints_);
			Mbar_dot_.resize(n_joints_);
			Ramda_.resize(n_joints_);
			Alpha_.resize(n_joints_);
			Omega_.resize(n_joints_);

			Xr_dot_ = 0.0;
			Xe_dot_ = 0.0;
			Xe_ddot_ = 0.0;
			Fd_ = 0.0;
			Fd_temp_ = 0.0;
			Fd_old_ = 0.0;
			Fe_ = 0.0;
			Fe_old_ = 0.0;
			M_ = 0.0;
			B_ = 0.0;
			del_B_ = 0.0;
			B_buffer_ = 0.0;
			PI_ = 0.0;
			PI_old_ = 0.0;

			filt_old_ = 0.0;
			filt_ = 0.0;
			tau_ = 1.0/(2*PI*9.0);

			f_cur_buffer_ = 0.0;

			experiment_mode_ = 0;

			std::vector<double> Mbar(n_joints_), Ramda(n_joints_), Alpha(n_joints_), Omega(n_joints_);
			for (size_t i=0; i<n_joints_; i++)
			{
				std::string si = boost::lexical_cast<std::string>(i+1);
				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/mbar", Mbar[i]) )
				{
					Mbar_(i) = Mbar[i];
				}
				else
				{
					std::cout << "/elfin/adaptive_impedance_controller/joint" + si + "/tdc/mbar" << std::endl;
					ROS_ERROR("Cannot find tdc/mbar gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/r", Ramda[i]) )
				{
					Ramda_(i) = Ramda[i];
				}
				else
				{
					ROS_ERROR("Cannot find tdc/r gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/a", Alpha[i]) )
				{
					Alpha_(i) = Alpha[i];
				}
				else
				{
					ROS_ERROR("Cannot find tdc/a gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/w", Omega[i]) )
				{
					Omega_(i) = Omega[i];
				}
				else
				{
					ROS_ERROR("Cannot find tdc/w gain");
					return false;
				}
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/fd", Fd_temp_))
			{
				ROS_ERROR("Cannot find aci/fd");
				return false;
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/m", M_))
			{
				ROS_ERROR("Cannot find aci/m");
				return false;
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/b", B_))
			{
				ROS_ERROR("Cannot find aci/b");
				return false;
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/mode", experiment_mode_))
			{
				ROS_ERROR("Cannot find mode");
				return false;
			}

			// command
			sub_q_cmd_ = n.subscribe("command", 1, &AdaptiveImpedanceController::commandCB, this);
			sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/elfin/elfin/ft_sensor_topic", 1, &AdaptiveImpedanceController::updateFTsensor, this);

			pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

   			return true;
  		}

		void starting(const ros::Time& time)
		{
			// get joint positions
			for(size_t i=0; i<n_joints_; i++) 
			{
				ROS_INFO("JOINT %d", (int)i);
				q_(i) = joints_[i].getPosition();
				q_init_(i) = q_(i);
				qdot_(i) = joints_[i].getVelocity();
			}

			time_ = 0.0;
			total_time_ = 0.0;

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
			f_cur_buffer_ = f_meas.force.y;
			f_cur_[2] = f_meas.force.z;
			f_cur_[3] = f_meas.torque.x;
			f_cur_[4] = f_meas.torque.y;
			f_cur_[5] = f_meas.torque.z;

			if (experiment_mode_ == 1 || experiment_mode_ == 2)
				f_cur_[1] = first_order_lowpass_filter();
		}

		// load gain is not permitted during controller loading?
		void loadGainCB()
		{
			
		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			// simple trajectory interpolation from joint command setpoint
			dt_ = period.toSec();

			if(total_time_ < 5.0)
			{
				task_init();	
			}
			else if(total_time_ >= 5.0 && total_time_ < 6.0)
			{
				task_via();
			}
			else if(total_time_ >= 6.0 && total_time_ < 16.0)
			{
				task_ready();
			}
			else if (total_time_ >= 16.0 && total_time_ < 17.0)
			{
				task_via();
			}
			else if (total_time_ >= 17.0 && total_time_ < 27.0)
			{
				task_freespace();
			}
			else if (total_time_ >= 27.0 && total_time_ < 28.0)
			{
				task_via();
			}
			else if (total_time_ >= 28.0 && total_time_ < 48.0)
			{
				task_contactspace();
			}
			else if (total_time_ >= 48.0 && total_time_ < 49.0)
			{
				task_via();
			}
			else if (total_time_ >= 49.0 && total_time_ < 59.0)
			{
				task_homming();
			}

			// get joint states
			for (size_t i=0; i<n_joints_; i++)
			{
				q_(i) = joints_[i].getPosition();
				qdot_(i) = joints_[i].getVelocity();
			}

			qddot_.data = (qdot_.data - qdot_old_.data) / dt_;

			// error
			KDL::JntArray q_error;
			KDL::JntArray q_error_dot;
			KDL::JntArray ded;
			KDL::JntArray tde;
			KDL::JntArray s;

			q_error.data = Eigen::VectorXd::Zero(n_joints_);
			q_error_dot.data = Eigen::VectorXd::Zero(n_joints_);
			ded.data = Eigen::VectorXd::Zero(n_joints_);
			tde.data = Eigen::VectorXd::Zero(n_joints_);
			s.data = Eigen::VectorXd::Zero(n_joints_);

			double db = 0.001;

			q_error.data = q_cmd_.data - q_.data;
			q_error_dot.data = qdot_cmd_.data - qdot_.data;

			for(size_t i=0; i<n_joints_; i++)
			{
				s(i) = q_error_dot(i) + Ramda_(i)*q_error(i);

				if(db < Mbar_(i) && Mbar_(i) > Ramda_(i)/Alpha_(i))
				{
					Mbar_dot_(i) = Alpha_(i)*s(i)*s(i) - Alpha_(i)*Omega_(i)*Mbar_(i);
				}

				if(Mbar_(i) < db)
				{
					Mbar_(i) = db;
				}
				
				if(Mbar_(i) > Ramda_(i)/Alpha_(i))
				{
					Mbar_(i) = Ramda_(i)/Alpha_(i) - db;
				}

				Mbar_(i) = Mbar_(i) + dt_*Mbar_dot_(i);
			}

			// torque command
			for(size_t i=0; i<n_joints_; i++)
			{
				ded(i) = qddot_cmd_(i) + 2.0 * Ramda_(i)*q_error(i) + Ramda_(i)*Ramda_(i)*q_error_dot(i);
				tde(i) = tau_cmd_old_(i) - Mbar_(i)*qddot_(i);
				tau_cmd_(i) = Mbar_(i)*ded(i) + tde(i);
			}

			for(size_t i=0; i<n_joints_; i++)
			{
				joints_[i].setCommand(tau_cmd_(i));
			}

			tau_cmd_old_.data = tau_cmd_.data;
			qdot_old_.data = qdot_.data;

			KDL::Frame Xdes;

			fk_solver_->JntToCart(q_cmd_, Xdes);

			SaveData_[0] = total_time_;
			SaveData_[1] = 0.122;
			SaveData_[2] = Xdes.p(2);
			SaveData_[3] = Fd_;
			SaveData_[4] = f_cur_(1);
			SaveData_[5] = f_cur_buffer_;
			SaveData_[6] = B_buffer_;

			msg_SaveData_.data.clear();

			for (size_t i = 0; i < SaveDataMax; i++)
			{
				msg_SaveData_.data.push_back(SaveData_[i]);
			}

			pub_SaveData_.publish(msg_SaveData_);

			time_ = time_ + dt_;
			total_time_ = total_time_ + dt_;				
  		}

  		void stopping(const ros::Time& time) { }

		void task_via()
		{
			time_ = 0.0;

			for (size_t i = 0; i < n_joints_; i++)
			{
				q_init_(i) = joints_[i].getPosition();
				q_cmd_(i) = q_cmd_end_(i);
				qdot_cmd_(i) = 0.0;
				qddot_cmd_(i) = 0.0;
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

			q_cmd_end_.data = q_cmd_.data;
		}

		void task_ready()
		{
			for (size_t i=0; i<n_joints_; i++)
			{
				if (i == 2 || i == 4)
				{
					q_cmd_(i) = trajectory_generator_pos(q_init_(i), PI/2.0, 10.0);
					qdot_cmd_(i) = trajectory_generator_vel(q_init_(i), PI/2.0, 10.0);
					qddot_cmd_(i) = trajectory_generator_acc(q_init_(i), PI/2.0, 10.0);
				}
				else
				{
					q_cmd_(i) = q_init_(i);
					qdot_cmd_(i) = 0.0;
					qddot_cmd_(i) = 0.0;
				}
			}

			q_cmd_end_.data = q_cmd_.data;
		}

		void task_freespace()
		{
			KDL::Frame start;
			KDL::Twist target_vel;
			KDL::JntArray cart_cmd;

			cart_cmd.data = Eigen::VectorXd::Zero(3);

			fk_solver_->JntToCart(q_init_, start);

			for (size_t i=0; i<6; i++)
			{
				if(i == 2)
				{
					target_vel(i) = trajectory_generator_vel(start.p(i), 0.122, 10.0);
				}
				else
				{
					target_vel(i) = 0.0;
				}
			}

			ik_vel_solver_->CartToJnt(q_cmd_, target_vel, qdot_cmd_);

			for (size_t i=0; i<n_joints_; i++)
			{
				q_cmd_(i) = q_cmd_(i) + qdot_cmd_(i)*dt_;
				qddot_cmd_(i) = (qdot_cmd_(i) - qdot_cmd_old_(i))/dt_;
				qdot_cmd_old_(i) = qdot_cmd_(i);
			}

			q_cmd_end_.data = q_cmd_.data;			
		}

		void task_contactspace()
		{
			KDL::Frame start;

			if (experiment_mode_ == 0 || experiment_mode_ == 1)
				Fd_ = Fd_temp_;
			else if (experiment_mode_ == 2)
				Fd_ = Fd_temp_ * 1 / 5 * sin(2 * PI * 0.2 * total_time_) + Fd_temp_;

				fk_solver_->JntToCart(q_init_, start);

			for (size_t i = 0; i < 6; i++)
			{
				if (i == 0)
				{
					Xc_dot_(i) = trajectory_generator_vel(start.p(i), 0.5, 20.0);
				}
				else if (i == 2)
				{
					Fe_ = f_cur_[1];
					PI_ = PI_old_ + dt_ * (Fd_old_ - Fe_old_) / B_;
					B_buffer_ = B_ + del_B_;
					Xc_ddot_(i) = 1/M_*((Fe_ - Fd_) - (B_ + del_B_)*Xc_dot_old_(i));
					Xc_dot_(i) = Xc_dot_old_(i) + Xc_ddot_(i)*dt_;
					del_B_ = B_ / Xc_dot_(i) * PI_;
					Xc_dot_old_(i) = Xc_dot_(i);
					Fd_old_ = Fd_;
					Fe_old_ = Fe_;
					PI_old_ = PI_;
				}
				else
				{
					Xc_ddot_(i) = 0.0;
					Xc_dot_(i) = 0.0;
				}
			}
			
			ik_vel_solver_->CartToJnt(q_cmd_, Xc_dot_, qdot_cmd_);

			for (size_t i=0; i<n_joints_; i++)
			{
				q_cmd_(i) = q_cmd_(i) + qdot_cmd_(i)*dt_;
				qddot_cmd_(i) = (qdot_cmd_(i) - qdot_cmd_old_(i))/dt_;
				qdot_cmd_old_(i) = qdot_cmd_(i);
			}

			q_cmd_end_.data = q_cmd_.data;
		}

		void task_homming()
		{
			for (size_t i = 0; i < n_joints_; i++)
			{
				q_cmd_(i) = trajectory_generator_pos(q_init_(i), 0.0, 10.0);
				qdot_cmd_(i) = trajectory_generator_vel(q_init_(i), 0.0, 10.0);
				qddot_cmd_(i) = trajectory_generator_acc(q_init_(i), 0.0, 10.0);
			}

			q_cmd_end_.data = q_cmd_.data;
		}

		double trajectory_generator_pos(double dStart, double dEnd, double dDuration)
		{
			double dA0 = dStart;
			double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
			double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
			double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

			return dA0 + dA3*time_*time_*time_ + dA4*time_*time_*time_*time_ + dA5*time_*time_*time_*time_*time_;
		}

		double trajectory_generator_vel(double dStart, double dEnd, double dDuration)
		{
			double dA0 = dStart;
			double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
			double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
			double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

			return 3.0*dA3*time_*time_ + 4.0*dA4*time_*time_*time_ + 5.0*dA5*time_*time_*time_*time_;
		}

		double trajectory_generator_acc(double dStart, double dEnd, double dDuration)
		{
			double dA0 = dStart;
			double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
			double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
			double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

			return 6.0*dA3*time_ + 12.0*dA4*time_*time_ + 20.0*dA5*time_*time_*time_;
		}		

		double first_order_lowpass_filter()
		{
			filt_ = (tau_ * filt_old_ + dt_*f_cur_buffer_)/(tau_ + dt_);
			filt_old_ = filt_;

			return filt_;
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
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	
		boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
		KDL::JntArray G_; 
		KDL::Vector gravity_;

		// tdc gain
		KDL::JntArray Mbar_, Mbar_dot_, Ramda_, Alpha_, Omega_;					

		// cmd, state
		KDL::JntArray q_init_;
		KDL::JntArray tau_cmd_, tau_cmd_old_;
		KDL::JntArray q_cmd_, q_cmd_old_, qdot_cmd_, qdot_cmd_old_, qddot_cmd_;
		KDL::JntArray q_cmd_end_;
		KDL::JntArray q_cmd_sp_;
		KDL::JntArray q_, qdot_, qdot_old_, qddot_;
		KDL::Wrench f_cur_;
		double f_cur_buffer_;

		KDL::Twist Xc_dot_, Xc_dot_old_, Xc_ddot_;

		double Xr_dot_, Xe_dot_;
		double Xe_ddot_;

		double Fd_, Fd_temp_, Fd_old_, Fe_, Fe_old_;
		double M_, B_, del_B_, B_buffer_;
		double PI_, PI_old_;

		double dt_;
		double time_;
		double total_time_;

		double filt_old_;
		double filt_;
		double tau_;

		int experiment_mode_;

		// topic
		ros::Subscriber sub_q_cmd_;
		ros::Subscriber sub_forcetorque_sensor_;

		double SaveData_[SaveDataMax];

		ros::Publisher pub_SaveData_;

		std_msgs::Float64MultiArray msg_SaveData_;			
	};
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdaptiveImpedanceController, controller_interface::ControllerBase)

