// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/pid.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
// #include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

//
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include "arm_controllers/TaskSpaceControllerJointState.h"
#include "arm_controllers/ComputedTorqueControllerCLIKParamsConfig.h"

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 97
#define num_taskspace 6
#define A 0.1
#define b -0.3
#define f 1
#define t_set 5

namespace arm_controllers
{
class ComputedTorqueControllerCLIK : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
    class GainsHandler
    {
      public:
        struct Gains
        {
            Gains() : K_Regulation_(0.0), K_Tracking_(0.0) {}
            
            Gains(double K_Regulation, double K_Tracking) : K_Regulation_(K_Regulation), K_Tracking_(K_Tracking) {}

            double K_Regulation_; 
            double K_Tracking_;
        };

        GainsHandler() {}

        bool initDynamicReconfig(const ros::NodeHandle &node)
        {
            ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

            Gains gains;
            
            if (!node.getParam("K_Regulation", gains.K_Regulation_))
            {
                ROS_ERROR("Could not find K_Regulation gain in %s", (node.getNamespace() + "/K_Regulation").c_str());
                return false;
            }

            if (!node.getParam("K_Tracking", gains.K_Tracking_))
            {
                ROS_ERROR("Could not find K_Tracking gain in %s", (node.getNamespace() + "/K_Tracking").c_str());
                return false;
            }

            // Start dynamic reconfigure server
            param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
            dynamic_reconfig_initialized_ = true;

            setGains(gains);

            // Set Dynamic Reconfigure's gains to clik gains
            updateDynamicReconfig();

            // Set callback
            param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
            param_reconfig_server_->setCallback(param_reconfig_callback_);

            return true;
        }

        void getGains(double &K_Regulation, double &K_Tracking)
        {
            Gains gains = *gains_buffer_.readFromRT();
            K_Regulation = gains.K_Regulation_;
            K_Tracking = gains.K_Tracking_;
        }

        Gains getGains()
        {
            return *gains_buffer_.readFromRT();
        }

        void setGains(double K_Regulation, double K_Tracking)
        {
            Gains gains(K_Regulation, K_Tracking);

            setGains(gains);
        }

        void setGains(const Gains &gains)
        {
            gains_buffer_.writeFromNonRT(gains);

            updateDynamicReconfig(gains);
        }

        void updateDynamicReconfig()
        {
            // Make sure dynamic reconfigure is initialized
            if (!dynamic_reconfig_initialized_)
                return;

            // Get starting values
            ComputedTorqueControllerCLIKParamsConfig config;
            getGains(config.K_Regulation, config.K_Tracking);

            updateDynamicReconfig(config);
        }

        void updateDynamicReconfig(Gains gains)
        {
            // Make sure dynamic reconfigure is initialized
            if (!dynamic_reconfig_initialized_)
                return;

            ComputedTorqueControllerCLIKParamsConfig config;

            // Convert to dynamic reconfigure format
            config.K_Regulation = gains.K_Regulation_;
            config.K_Tracking = gains.K_Tracking_;

            updateDynamicReconfig(config);
        }

        void updateDynamicReconfig(ComputedTorqueControllerCLIKParamsConfig config)
        {
            // Make sure dynamic reconfigure is initialized
            if (!dynamic_reconfig_initialized_)
                return;

            // Set starting values, using a shared mutex with dynamic reconfig
            param_reconfig_mutex_.lock();
            param_reconfig_server_->updateConfig(config);
            param_reconfig_mutex_.unlock();
        }

        void dynamicReconfigCallback(ComputedTorqueControllerCLIKParamsConfig &config, uint32_t /*level*/)
        {
            ROS_DEBUG_STREAM_NAMED("CLIK gain", "Dynamics reconfigure callback recieved.");

            // Set the gains
            setGains(config.K_Regulation, config.K_Tracking);
        }

      private:
        // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
        // blocking the realtime update loop
        realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

        // Dynamics reconfigure
        bool dynamic_reconfig_initialized_;
        typedef dynamic_reconfigure::Server<arm_controllers::ComputedTorqueControllerCLIKParamsConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
        DynamicReconfigServer::CallbackType param_reconfig_callback_;

        boost::recursive_mutex param_reconfig_mutex_;
    };

  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // 추가 이건 뭐지?
        loop_count_ = 0;

        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.0 Control objective & Inverse Kinematics mode
        if (!n.getParam("ctr_obj", ctr_obj_))
        {
            ROS_ERROR("Could not find control objective");
            return false;
        }

        if (!n.getParam("ik_mode", ik_mode_))
        {
            ROS_ERROR("Could not find control objective");
            return false;
        }

        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 PID Gains
        pids_.resize(n_joints_);

        for (size_t i = 0; i < n_joints_; i++)
        {
            if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
            {
                ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
                return false;
            }
        }

        // 1.2.2 Closed-loop Inverse Kinematics Gains (총 3개: Reconfigure 한개, 여러개, 미적용)

        // // *** Dynamic Reconfiguration ver.1: 한개 *** //
		// if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/clik_gains")) )
		// {
		// 	ROS_ERROR_STREAM("Failed to load K_Regulation gain parameter from gains");
		// 	return false;
		// }
        // // *** Dynamic Reconfiguration ver.1: 한개 *** //


        // *** Dynamic Reconfiguration ver.2: 여러개  *** //
        for (size_t i = 0; i < n_joints_; i++)
        {
            GainsHandlerSharedPtr gains_handler_ptr(new GainsHandler());
            gains_handler_.push_back(gains_handler_ptr);
            // if (!gains_handler_[i].initDynamicReconfig(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/clik")) )
            if (!gains_handler_[i]->initDynamicReconfig(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/clik/")) )
		    {
			    ROS_ERROR_STREAM("Failed to load clik gain parameter from " << joint_names_[i] + "/clik");
			    return false;
		    }
        }
        // *** Dynamic Reconfiguration ver.2: 여러개  *** //

        // // *** no dynamic reconfiguration  *** //
        // K_reg_.resize(n_joints_);
        // K_track_.resize(n_joints_);

        // std::vector<double> K_reg(n_joints_), K_track(n_joints_);

        // for (size_t i=0; i<n_joints_; i++)
		// {
		// 	if (n.getParam("gains/" + joint_names_[i] + "/clik/K_Regulation",K_reg[i] ))
        //     {
        //         K_reg_(i) = K_reg[i];
        //     }
        //     else
		// 	{
		// 		ROS_ERROR("Could not find p gain in %s", ("gains/"+joint_names_[i]+"/clik/K_Regulation").c_str());
		// 		return false;
		// 	}

		// 	if (n.getParam("gains/" + joint_names_[i] + "/clik/K_Tracking",K_track[i] ))
        //     {
        //         K_track_(i) = K_track[i];
        //     }
        //     else
		// 	{
		// 		ROS_ERROR("Could not find p gain in %s", ("gains/"+joint_names_[i]+"/clik/K_Tracking").c_str());
		// 		return false;
		// 	}

		// }
        // // *** no dynamic reconfiguration *** //

        // 2. ********* urdf *********
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

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
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

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
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
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        // 4.4 jacobian solver 초기화
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // 4.5 forward kinematics solver 초기화
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);
        tau_pid_.data = Eigen::VectorXd::Zero(n_joints_);
        x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 KDL Matrix 초기화 (사이즈 정의 및 값 0)
        J_.resize(kdl_chain_.getNrOfJoints());
        Ja_.resize(kdl_chain_.getNrOfJoints()); // 6x6 or 6x7?

        // J_inv_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());

        

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        // 6.1.1
        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.1.2
        controller_state_pub_.reset(
            new realtime_tools::RealtimePublisher<arm_controllers::TaskSpaceControllerJointState>(n, "state", 1));

        controller_state_pub_->msg_.header.stamp = ros::Time::now();
        for (size_t i = 0; i < n_joints_; i++)
        {
            controller_state_pub_->msg_.name.push_back(joint_names_[i]);
            controller_state_pub_->msg_.qd.push_back(0.0);
            controller_state_pub_->msg_.qd_dot.push_back(0.0);
            controller_state_pub_->msg_.q.push_back(0.0);
            controller_state_pub_->msg_.q_dot.push_back(0.0);
            controller_state_pub_->msg_.e.push_back(0.0);
            controller_state_pub_->msg_.e_dot.push_back(0.0);
            controller_state_pub_->msg_.xd.push_back(0.0);
            controller_state_pub_->msg_.xd_dot.push_back(0.0);
            controller_state_pub_->msg_.x.push_back(0.0);
            controller_state_pub_->msg_.x_dot.push_back(0.0);
            controller_state_pub_->msg_.ex.push_back(0.0);
            controller_state_pub_->msg_.ex_dot.push_back(0.0);
            controller_state_pub_->msg_.effort_total.push_back(0.0);
            controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
            controller_state_pub_->msg_.effort_feedback.push_back(0.0);
        }

        // 6.2 subsriber
        sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &ComputedTorqueControllerCLIK::commandCB, this);
        event = 0; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1

        ROS_INFO("end of init");

        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != num_taskspace)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
            return;
        }

        for (int i = 0; i < num_taskspace; i++)
        {
            x_cmd_(i) = msg->data[i];
        }

        event = 1; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        // 0.3 end-effector state by Compute forward kinematics (x_, xdot_)
        fk_pos_solver_->JntToCart(q_, x_); // position vector (3x1) + rotation matrix (3x3)
        xdot_ = J_.data * qdot_.data;

        // ********* 1. Desired Trajectory Generation in task space *********

        // *** 1.1 Desired Trajectory in taskspace ***
        if (ctr_obj_ == 1)
        {
            // (1) Set Point Regulation
            if (event == 0) // initial command
            {
                xd_.p(0) = 0.0;
                xd_.p(1) = 0.0;
                xd_.p(2) = 0.4;
                xd_.M = KDL::Rotation::EulerZYX(0,0,0);
            }
            else if (event == 1) // command from ros subscriber
            {
                xd_.p(0) = x_cmd_(0);
                xd_.p(1) = x_cmd_(1);
                xd_.p(2) = x_cmd_(2);
                xd_.M = KDL::Rotation::EulerZYX(x_cmd_(3), x_cmd_(4), x_cmd_(5)); 
            }

            xd_dot_(0) = 0;
            xd_dot_(1) = 0;
            xd_dot_(2) = 0;
            xd_dot_(3) = 0;
            xd_dot_(4) = 0;
            xd_dot_(5) = 0;

            xd_ddot_(0) = 0;
            xd_ddot_(1) = 0;
            xd_ddot_(2) = 0;
            xd_ddot_(3) = 0;
            xd_ddot_(4) = 0;
            xd_ddot_(5) = 0;
        }
        else if (ctr_obj_ == 2)
        {
            // (2) Tracking
            xd_.p(0) = 0;
            xd_.p(1) = A * sin(f * M_PI * (t - t_set)) + b;
            xd_.p(2) = 0.3;
            xd_.M = KDL::Rotation::EulerZYX(0, 0, 0);


            xd_dot_(0) = 0;
            xd_dot_(1) = (f * M_PI) * A * cos(f * M_PI * (t - t_set));
            xd_dot_(2) = 0;
            xd_dot_(3) = 0;
            xd_dot_(4) = 0;
            xd_dot_(5) = 0;

            xd_ddot_(0) = 0;
            xd_ddot_(1) = -1 * (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - t_set));
            xd_ddot_(2) = 0;
            xd_ddot_(3) = 0;
            xd_ddot_(4) = 0;
            xd_ddot_(5) = 0;
        }

        // ********* 2. Inverse Kinematics *********
        // *** 2.0 Error Definition in Task Space ***

        // ***************** 수정중 *****************
        // // ver. 01: original
        // ex_temp_ = diff(x_, xd_);

        // ex_(0) = ex_temp_(0);
        // ex_(1) = ex_temp_(1);
        // ex_(2) = ex_temp_(2);
        // ex_(3) = ex_temp_(3);
        // ex_(4) = ex_temp_(4);
        // ex_(5) = ex_temp_(5);

        // ver. 02: revise
        // define position error
        ex_p_ = xd_.p - x_.p;

        // define orientation error (Euler ZYX = Fixed XYZ)
        xd_.M.GetEulerZYX(alpha_d_,beta_d_,gamma_d_);
        x_.M.GetEulerZYX(alpha_,beta_,gamma_);

        ex_o_(0) = alpha_d_ - alpha_;
        ex_o_(1) = beta_d_ - beta_;
        ex_o_(2) = gamma_d_ - gamma_; 

        // 
        for (size_t i = 0; i < num_taskspace; i++)
        {
            if (i<3)
            {
                ex_(i) = ex_p_(i);
            }
            else
            {
                ex_(i) = ex_o_(i-3);
            }

        }

        ex_dot_ = xd_dot_ - xdot_;

        // new version(2.1~2.2) //

        // *** 2.1 Computing Analytic Jacobian Ja(q) ***
        // 2.1.0 Computing Geometric Jacobian J(q) = {Jv;Jw};
        jnt_to_jac_solver_->JntToJac(q_, J_);
        
        Jv_ = J_.data.block(0,0,3,6);
        Jw_ = J_.data.block(3,0,3,6);

        // 2.1.1 Creating 3x3 rotation matrix for transform geometric jacobian to analytic jacobian from roll, pitch, yaw of current end-effector 
        // Euler ZYX
        Eigen::Matrix<double, 3, 3> T_;
        T_(0,0) = 0.0;
        T_(0,1) = -sin(alpha_);
        T_(0,2) = cos(alpha_)*cos(beta_);
        T_(1,0) = 0.0;
        T_(1,1) = cos(alpha_);
        T_(1,2) = sin(alpha_)*cos(beta_);
        T_(2,0) = 1.0;
        T_(2,1) = 0.0;
        T_(2,2) = -sin(beta_);

        // 2.1.2 Computing Analytic Jacobian from Geometic Jacobian
        Ja_.data.block(0,0,3,6) = Jv_;
        Ja_.data.block(3,0,3,6) = T_.inverse() * Jw_;

        // *** 2.2 Computing Analytic Jacobian transpose/inversion ***
        Ja_transpose_ = Ja_.data.transpose();
        Ja_inv_ = Ja_.data.inverse();


        // old version(2.1~2.2) //
        // *** 2.1 computing Geometric Jacobian J(q) ***
        jnt_to_jac_solver_->JntToJac(q_, J_);

        // *** 2.2 computing Jacobian transpose/inversion ***
        J_transpose_ = J_.data.transpose();
        J_inv_ = J_.data.inverse();

        // *** 2.3 computing desired joint state from Open-loop/Closed-loop Inverse Kinematics ***
        // 2.3.0 CLIK Gains update from server using dynamic reconfiguration

        // // *** Dynamic Reconfiguration ver.1: 한개 *** //
        // GainsHandler::Gains gains = gains_handler_.getGains();
        // // *** Dynamic Reconfiguration ver.1: 한개 *** //

        // *** Dynamic Reconfiguration ver.2: 여러개*** //
        K_reg_.resize(n_joints_);
        K_track_.resize(n_joints_);
        // std::vector<GainsHandler::Gains> gains;
        
        // for (size_t i = 0; i < n_joints_; i++)
        // {
            // gains[i] = gains_handler_[i].getGains();
            // gains[i] = gains_handler_[i]->getGains();
        // }

        for (size_t i = 0; i < n_joints_; i++)
        {
            gains_handler_[i]->getGains(K_reg_(i), K_track_(i));
            // K_Regulation_;
            // K_track_(i) = gains[i].K_Tracking_;
        }

        // *** Dynamic Reconfiguration ver.2: 여러개*** //

        // 2.3.1 Regulation Case
        if (ctr_obj_ == 1)
        {

            if (t < t_set)
            {
                qd_.data = qd_old_.data;
            }
            else
            {
                // // *** Dynamic Reconfiguration ver.1: 한개 *** //
                // qd_.data = qd_old_.data + Ja_inv_ * gains.K_Regulation_ * ex_ * dt;
                // // *** Dynamic Reconfiguration ver.1: 한개 *** //

                // *** Dynamic Reconfiguration ver.2: 여러개 *** //
                qd_.data = qd_old_.data + Ja_transpose_ * K_reg_.data.cwiseProduct(ex_) * dt;
                // *** Dynamic Reconfiguration ver.2: 여러개 *** //

                qd_old_.data = qd_.data;
            }
        }
        // 2.3.2 Tracking Case
        else if (ctr_obj_ == 2)
        {
            if (t < t_set)
            {
                qd_.data = qd_old_.data;
            }
            else
            {
                if (ik_mode_ == 1) // Open-loop Inverse Kinematics
                {
                    qd_.data = qd_old_.data + Ja_inv_ * xd_dot_ * dt;
                    qd_old_.data = qd_.data;
                }
                else if (ik_mode_ == 2) // Closed-loop Inverse Kinematics
                {
                    // // *** Dynamic Reconfiguration ver.1: 한개 *** //
                    // qd_.data = qd_old_.data + Ja_inv_ * (xd_dot_ + gains.K_Tracking_* ex_) * dt;
                    // // *** Dynamic Reconfiguration ver.1: 한개 *** //

                    // *** Dynamic Reconfiguration ver.2: 여러개 *** //
                    qd_.data = qd_old_.data + Ja_inv_ * (xd_dot_ + K_track_.data.cwiseProduct(ex_)) * dt;
                    // *** Dynamic Reconfiguration ver.2: 여러개 *** //
                    qd_old_.data = qd_.data;
                }
            }
        }

        // ********* 3. Motion Controller in Joint Space*********
        // *** 3.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;

        // *** 3.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); // output은 머지? , id_solver는 어디에서?

        // *** 3.3 Apply Torque Command to Actuator ***
        for (size_t i = 0; i < n_joints_; i++)
        {
            tau_pid_(i) = pids_[i].computeCommand(e_(i), e_dot_(i), period);
        }

        aux_.data = M_.data * (qd_ddot_.data + tau_pid_.data);
        comp_.data = C_.data + G_.data;
        tau_d_.data = aux_.data + comp_.data;

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
            // joints_[i].setCommand(0.0);
        }

        // ********* 4. data 저장 *********
        save_data();

        // ********* 5. state 출력 *********
        print_state();

        // ********* 6. ROS Publisher *********
        // 6.1 data for matlab plot and printf
        // 6.1.1
        msg_SaveData_.data.clear();

        // 6.1.2
        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 6.1.3
        pub_SaveData_.publish(msg_SaveData_);

        // 6.2 data for rqt_gui
        if (loop_count_ % 10 == 0) // loop_counts는 머지?
        {
            if (controller_state_pub_->trylock())
            {
                controller_state_pub_->msg_.header.stamp = time;
                for (int i = 0; i < n_joints_; i++)
                {
                    controller_state_pub_->msg_.qd[i] = R2D * qd_(i);
                    controller_state_pub_->msg_.qd_dot[i] = R2D * qd_dot_(i);
                    controller_state_pub_->msg_.q[i] = R2D * q_(i);
                    controller_state_pub_->msg_.q_dot[i] = R2D * qdot_(i);
                    controller_state_pub_->msg_.e[i] = R2D * e_(i);
                    controller_state_pub_->msg_.e_dot[i] = R2D * e_dot_(i);
                }


                // To do # 1

                // Position of End-effector (unit: mm, mm/s) // velocity update 해야함
                for (int i = 0; i < 3; i++)
                {
                    controller_state_pub_->msg_.xd[i] = 1000 * xd_.p(i);
                    controller_state_pub_->msg_.xd_dot[i] = 1000 * xd_dot_(i);
                    controller_state_pub_->msg_.x[i] = 1000 * x_.p(i);
                    controller_state_pub_->msg_.x_dot[i] = 1000 * xdot_(i);
                    controller_state_pub_->msg_.ex[i] = 1000 * ex_(i);
                    controller_state_pub_->msg_.ex_dot[i] = 1000 * ex_dot_(i);
                }

                // Orientation of End-effector (unit: deg, deg/s) // 모두 업데이트 해야함
                for (int i = 3; i < 6; i++)
                {
                    controller_state_pub_->msg_.xd[i] = 0.0;
                    controller_state_pub_->msg_.xd_dot[i] = 0.0;
                    controller_state_pub_->msg_.x[i] = 0.0;
                    controller_state_pub_->msg_.x_dot[i] = 0.0;
                    controller_state_pub_->msg_.ex[i] = 0.0;
                    controller_state_pub_->msg_.ex_dot[i] = 0.0;
                }
                // for (int i = 3; i < 6; i++)
                // {
                //     controller_state_pub_->msg_.xd[i] = R2D * xd_(i);
                //     controller_state_pub_->msg_.xd_dot[i] = R2D * xd_dot_(i);
                //     controller_state_pub_->msg_.x[i] = R2D * x_(i);
                //     controller_state_pub_->msg_.x_dot[i] = R2D * xdot_(i);
                //     controller_state_pub_->msg_.ex[i] = R2D * ex_(i);
                //     controller_state_pub_->msg_.ex_dot[i] = R2D * ex_dot_(i);
                // }


                // To do # 1
                for (int i = 0; i < n_joints_; i++)
                {
                    controller_state_pub_->msg_.effort_total[i] = tau_d_(i);
                    controller_state_pub_->msg_.effort_feedforward[i] = comp_(i);
                    controller_state_pub_->msg_.effort_feedback[i] = aux_(i);
                }
                controller_state_pub_->unlockAndPublish();
            }
        }
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // // 1
        // // Simulation time (unit: sec)
        // SaveData_[0] = t;

        // // Desired position in joint space (unit: rad)
        // SaveData_[1] = qd_(0);
        // SaveData_[2] = qd_(1);
        // SaveData_[3] = qd_(2);
        // SaveData_[4] = qd_(3);
        // SaveData_[5] = qd_(4);
        // SaveData_[6] = qd_(5);

        // // Desired velocity in joint space (unit: rad/s)
        // SaveData_[7] = qd_dot_(0);
        // SaveData_[8] = qd_dot_(1);
        // SaveData_[9] = qd_dot_(2);
        // SaveData_[10] = qd_dot_(3);
        // SaveData_[11] = qd_dot_(4);
        // SaveData_[12] = qd_dot_(5);

        // // Desired acceleration in joint space (unit: rad/s^2)
        // SaveData_[13] = qd_ddot_(0);
        // SaveData_[14] = qd_ddot_(1);
        // SaveData_[15] = qd_ddot_(2);
        // SaveData_[16] = qd_ddot_(3);
        // SaveData_[17] = qd_ddot_(4);
        // SaveData_[18] = qd_ddot_(5);

        // // Actual position in joint space (unit: rad)
        // SaveData_[19] = q_(0);
        // SaveData_[20] = q_(1);
        // SaveData_[21] = q_(2);
        // SaveData_[22] = q_(3);
        // SaveData_[23] = q_(4);
        // SaveData_[24] = q_(5);

        // // Actual velocity in joint space (unit: rad/s)
        // SaveData_[25] = qdot_(0);
        // SaveData_[26] = qdot_(1);
        // SaveData_[27] = qdot_(2);
        // SaveData_[28] = qdot_(3);
        // SaveData_[29] = qdot_(4);
        // SaveData_[30] = qdot_(5);

        // // Error position in joint space (unit: rad)
        // SaveData_[31] = e_(0);
        // SaveData_[32] = e_(1);
        // SaveData_[33] = e_(2);
        // SaveData_[34] = e_(3);
        // SaveData_[35] = e_(4);
        // SaveData_[36] = e_(5);

        // // Error velocity in joint space (unit: rad/s)
        // SaveData_[37] = e_dot_(0);
        // SaveData_[38] = e_dot_(1);
        // SaveData_[39] = e_dot_(3);
        // SaveData_[40] = e_dot_(4);
        // SaveData_[41] = e_dot_(5);
        // SaveData_[42] = e_dot_(6);

        // // Desired position in task space (unit: m)
        // SaveData_[49] = xd_(0);
        // SaveData_[50] = xd_(1);
        // SaveData_[51] = xd_(2);
        // SaveData_[52] = xd_(3);
        // SaveData_[53] = xd_(4);
        // SaveData_[54] = xd_(5);

        // // Desired velocity in task space (unit: m/s)
        // SaveData_[55] = xd_dot_(0);
        // SaveData_[56] = xd_dot_(1);
        // SaveData_[57] = xd_dot_(2);
        // SaveData_[58] = xd_dot_(3);
        // SaveData_[59] = xd_dot_(4);
        // SaveData_[60] = xd_dot_(5);

        // // Desired acceleration in task space (unit: m/s^2)
        // SaveData_[61] = xd_ddot_(0);
        // SaveData_[62] = xd_ddot_(1);
        // SaveData_[63] = xd_ddot_(2);
        // SaveData_[64] = xd_ddot_(3);
        // SaveData_[65] = xd_ddot_(4);
        // SaveData_[66] = xd_ddot_(5);

        // // Actual position in task space (unit: m)
        // SaveData_[67] = x_(0);
        // SaveData_[68] = x_(1);
        // SaveData_[69] = x_(0);
        // SaveData_[70] = x_(1);
        // SaveData_[71] = x_(0);
        // SaveData_[72] = x_(1);

        // // Actual velocity in task space (unit: m/s)
        // SaveData_[73] = xdot_(0);
        // SaveData_[74] = xdot_(1);
        // SaveData_[75] = xdot_(2);
        // SaveData_[76] = xdot_(3);
        // SaveData_[77] = xdot_(4);
        // SaveData_[78] = xdot_(5);

        // // Error position in task space (unit: m)
        // SaveData_[79] = ex_(0);
        // SaveData_[80] = ex_(1);
        // SaveData_[81] = ex_(2);
        // SaveData_[82] = ex_(3);
        // SaveData_[83] = ex_(4);
        // SaveData_[84] = ex_(5);

        // // Error velocity in task space (unit: m/s)
        // SaveData_[85] = ex_dot_(0);
        // SaveData_[86] = ex_dot_(1);
        // SaveData_[87] = ex_dot_(2);
        // SaveData_[88] = ex_dot_(3);
        // SaveData_[89] = ex_dot_(4);
        // SaveData_[90] = ex_dot_(5);

        // // Error intergal value in task space (unit: m*sec)
        // SaveData_[91] = ex_int_(0);
        // SaveData_[92] = ex_int_(1);
        // SaveData_[93] = ex_int_(2);
        // SaveData_[94] = ex_int_(3);
        // SaveData_[95] = ex_int_(4);
        // SaveData_[96] = ex_int_(5);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Command from Subscriber in Task Space  ***\n");
            if (event == 0)
            {
                printf("No Active!!!\n");
            }
            else
            {
                printf("Active!!!\n");
            }
            // printf("x_cmd: %f, ", x_cmd_(0));
            // printf("y_cmd: %f, ", x_cmd_(1));
            // printf("z_cmd: %f, ", x_cmd_(2));
            // printf("r_cmd: %f, ", x_cmd_(3));
            // printf("p_cmd: %f, ", x_cmd_(4));
            // printf("y_cmd: %f\n", x_cmd_(5));
            // printf("\n");

            // printf("*** Desired Position in Joint Space (unit: deg) ***\n");
            // printf("qd(1): %f, ", qd_(0) * R2D);
            // printf("qd(2): %f, ", qd_(1) * R2D);
            // printf("qd(3): %f, ", qd_(2) * R2D);
            // printf("qd(4): %f, ", qd_(3) * R2D);
            // printf("qd(5): %f, ", qd_(4) * R2D);
            // printf("qd(6): %f\n", qd_(5) * R2D);
            // printf("\n");

            // printf("*** Actual Position in Joint Space (unit: deg) ***\n");
            // printf("q(1): %f, ", q_(0) * R2D);
            // printf("q(2): %f, ", q_(1) * R2D);
            // printf("q(3): %f, ", q_(2) * R2D);
            // printf("q(4): %f, ", q_(3) * R2D);
            // printf("q(5): %f, ", q_(4) * R2D);
            // printf("q(6): %f\n", q_(5) * R2D);
            // printf("\n");
            //
            // printf("*** Joint Space Error (unit: deg)  ***\n");
            // printf("q1: %f, ", R2D * e_(0));
            // printf("q2: %f, ", R2D * e_(1));
            // printf("q3: %f, ", R2D * e_(2));
            // printf("q4: %f, ", R2D * e_(3));
            // printf("q5: %f, ", R2D * e_(4));
            // printf("q6: %f\n", R2D * e_(5));
            // printf("\n");
            
            printf("*** Desired Position in Task Space (unit: mm) ***\n");
            printf("xd: %f, ", xd_.p(0) * 1000 );
            printf("yd: %f, ", xd_.p(1) * 1000 );
            printf("zd: %f\n", xd_.p(2) * 1000 );
            printf("\n");

            printf("*** Actual Position in Task Space (unit: mm) ***\n");
            printf("x: %f, ", x_.p(0) * 1000 );
            printf("y: %f, ", x_.p(1) * 1000 );
            printf("z: %f\n", x_.p(2) * 1000 );
            printf("\n");

            // printf("*** Desired Orientation in Task Space (unit: ??) ***\n");
            // printf("xd_(0): %f, ", xd_.M(KDL::Rotation::RPY(0));
            // printf("xd_(1): %f, ", xd_.M(KDL::Rotation::RPY(1));
            // printf("xd_(2): %f\n", xd_.M(KDL::Rotation::RPY(2));
            // printf("\n");

            // printf("*** Actual Orientation in Task Space (unit: ??) ***\n");
            // printf("x_(0): %f, ", x_.M(KDL::Rotation::RPY(0));
            // printf("x_(1): %f, ", x_.M(KDL::Rotation::RPY(1));
            // printf("x_(2): %f\n", x_.M(KDL::Rotation::RPY(2));
            // printf("\n");

            // printf("*** Desired Translation Velocity in Task Space (unit: m/s) ***\n");
            // printf("xd_dot: %f, ", xd_dot_(0));
            // printf("yd_dot: %f, ", xd_dot_(1));
            // printf("zd_dot: %f\n", xd_dot_(2));
            // printf("\n");

            // printf("*** Actual Translation Velocity in Task Space (unit: m/s) ***\n");
            // printf("xdot: %f, ", xdot_(0));
            // printf("ydot: %f  ", xdot_(1));
            // printf("zdot: %f\n", xdot_(2));
            // printf("\n");

            // printf("*** Desired Angular Velocity in Task Space (unit: rad/s) ***\n");
            // printf("rd_dot: %f, ", xd_dot_(3));
            // printf("pd_dot: %f, ", xd_dot_(4));
            // printf("yd_dot: %f\n", xd_dot_(5));
            // printf("\n");

            // printf("*** Actual Angular Velocity in Task Space (unit: rad/s) ***\n");
            // printf("r_dot: %f, ", xdot_(3));
            // printf("p_dot: %f  ", xdot_(4));
            // printf("y_dot: %f\n", xdot_(5));
            // printf("\n");

            // printf("*** Desired Rotation Matrix of end-effector ***\n");
            // printf("%f, ", xd_.M(0, 0));
            // printf("%f, ", xd_.M(0, 1));
            // printf("%f\n", xd_.M(0, 2));
            // printf("%f, ", xd_.M(1, 0));
            // printf("%f, ", xd_.M(1, 1));
            // printf("%f\n", xd_.M(1, 2));
            // printf("%f, ", xd_.M(2, 0));
            // printf("%f, ", xd_.M(2, 1));
            // printf("%f\n", xd_.M(2, 2));
            // printf("\n");

            // printf("*** Actual Rotation Matrix of end-effector ***\n");
            // printf("%f, ", x_.M(0, 0));
            // printf("%f, ", x_.M(0, 1));
            // printf("%f\n", x_.M(0, 2));
            // printf("%f, ", x_.M(1, 0));
            // printf("%f, ", x_.M(1, 1));
            // printf("%f\n", x_.M(1, 2));
            // printf("%f, ", x_.M(2, 0));
            // printf("%f, ", x_.M(2, 1));
            // printf("%f\n", x_.M(2, 2));
            // printf("\n");

            printf("*** Task Space Position Error (unit: mm) ***\n");
            printf("x: %f, ", ex_(0) * 1000);
            printf("y: %f, ", ex_(1) * 1000);
            printf("z: %f\n", ex_(2) * 1000);
            printf("\n");

            printf("*** Task Space Orientation Error in terms of Euler ZYX (unit: deg) ***\n");
            printf("z: %f, ", ex_(3) * R2D);
            printf("y: %f, ", ex_(4) * R2D);
            printf("x: %f\n", ex_(5) * R2D);
            printf("\n");


            // printf("*** Task Space Position Error Test(unit: ) ***\n");
            // printf("ex_temp(0): %f, ", ex_temp_(0));
            // printf("ex_temp(1): %f, ", ex_temp_(1));
            // printf("ex_temp(2): %f\n, ", ex_temp_(2));
            // printf("ex_temp(3): %f, ", ex_temp_(3));
            // printf("ex_temp(4): %f, ", ex_temp_(4));
            // printf("ex_temp(5): %f\n, ", ex_temp_(5));
            // printf("ex_temp(6): %f, ", ex_temp_(6));
            // printf("ex_temp(7): %f, ", ex_temp_(7));
            // printf("ex_temp(8): %f\n, ", ex_temp_(8));
            // printf("ex_temp(9): %f, ", ex_temp_(9));
            // printf("ex_temp(10): %f, ", ex_temp_(10));
            // printf("ex_temp(11): %f\n, ", ex_temp_(11));
            printf("\n");

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    int ctr_obj_;
    int ik_mode_;
    int event;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl and Eigen Jacobian
    KDL::Jacobian J_;  // Geometric Jacobian
    Eigen::Matrix<double, 3, num_taskspace> Jv_;
    Eigen::Matrix<double, 3, num_taskspace> Jw_;
    Eigen::MatrixXd J_inv_;
    Eigen::MatrixXd J_transpose_;

    KDL::Jacobian Ja_; // Analytic Jacobain
    Eigen::MatrixXd Ja_inv_;
    Eigen::MatrixXd Ja_transpose_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    // boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;               // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_;

    // Task Space State
    // ver. 01
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_;

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_;
    Eigen::Matrix<double, num_taskspace, 1> xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> xdot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_;

    // 
    KDL::Vector ex_p_;
    KDL::Vector ex_o_;
    double r_, p_, y_, rd_, pd_, yd_;
    double alpha_, beta_, gamma_, alpha_d_, beta_d_, gamma_d_;

    // Eigen::Matrix<double, 3, 1> ex_p_, ex_o_;

    // ver. 02
    // Eigen::Matrix<double, num_taskspace, 1> xd_, xd_dot_, xd_ddot_;
    // Eigen::Matrix<double, num_taskspace, 1> x_, xdot_;
    // KDL::Frame x_temp_;
    // Eigen::Matrix<double, num_taskspace, 1> ex_, ex_dot_, ex_int_;

    // Input
    KDL::JntArray x_cmd_;

    // Torque
    KDL::JntArray aux_;
    KDL::JntArray comp_;
    KDL::JntArray tau_pid_;
    KDL::JntArray tau_d_;

    // gains
    std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

    // // *** Dynamic Reconfiguration ver.1: 한개 *** //
    // GainsHandler gains_handler_;		     // K_Regulation gain(dynamic reconfigured)
    // // *** Dynamic Reconfiguration ver.1: 한개 *** //

    // *** Dynamic Reconfiguration ver.2: 여러개 *** //
    // std::vector<GainsHandler> gains_handler_;
    typedef boost::shared_ptr<GainsHandler> GainsHandlerSharedPtr;
    std::vector<GainsHandlerSharedPtr > gains_handler_;
    // std::vector<GainsHandler*> gains_handler_;
    // *** Dynamic Reconfiguration ver.1: 여러개 *** //

    KDL::JntArray K_reg_, K_track_;
    
    // save the data
    double SaveData_[SaveDataMax];

    // ros subscriber
    ros::Subscriber sub_x_cmd_;

    // ros publisher
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_SaveData_;

    // 추가 for making rqt_gui plot
    realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;

    ros::Subscriber command_sub_;
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            arm_controllers::TaskSpaceControllerJointState> >
        controller_state_pub_;

    int loop_count_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::ComputedTorqueControllerCLIK, controller_interface::ControllerBase)