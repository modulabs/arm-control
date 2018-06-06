#ifndef ARM_CONTROLLERS_PID_H_
#define ARM_CONTROLLERS_PID_H_

#include "arm_controllers/PidParamsConfig.h"

namespace arm_controllers{

class Pid
{
public:
    Pid()
        : p_(0.0), i_(0.0), d_(0.0)
    {}

    void set(double p, double i, double d)
    {
        p_ = p; i_ = i; d_ = d;
    }

    void get(double& p, double& i, double& d)
    {
        p = p_; i = i_; d = d_;
    }

    bool init(const ros::NodeHandle &n)
    {
        std::string node_name = n.getNamespace();
        if (!n.getParam("p", p_))
        {
            ROS_ERROR("Could not find p gain in %s", (node_name+"p").c_str());
            return false;
        }

        if (!n.getParam("i", i_))
        {
            ROS_ERROR("Could not find i gain in %s", (node_name+"i").c_str());
            return false;
        }

        if (!n.getParam("d", d_))
        {
            ROS_ERROR("Could not find d gain in %s", (node_name+"d").c_str());
            return false;
        }

        initDynamicReconfig(n);
        
        return true;
    }

    void initDynamicReconfig(const ros::NodeHandle &node)
    {
        ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

        // Start dynamic reconfigure server
        param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
        dynamic_reconfig_initialized_ = true;

        // Set Dynamic Reconfigure's gains to Pid's values
        updateDynamicReconfig();

        // Set callback
        param_reconfig_callback_ = boost::bind(&Pid::dynamicReconfigCallback, this, _1, _2);
        param_reconfig_server_->setCallback(param_reconfig_callback_);
    }

    void updateDynamicReconfig()
    {
        // Make sure dynamic reconfigure is initialized
        if(!dynamic_reconfig_initialized_)
            return;

        // Get starting values
        arm_controllers::PidParamsConfig config;

        // Get starting values
        get(config.p, config.i, config.d);

        updateDynamicReconfig(config);
    }

    void updateDynamicReconfig(PidParamsConfig config)
    {
        // Make sure dynamic reconfigure is initialized
        if(!dynamic_reconfig_initialized_)
            return;

        // Set starting values, using a shared mutex with dynamic reconfig
        param_reconfig_mutex_.lock();
        param_reconfig_server_->updateConfig(config);
        param_reconfig_mutex_.unlock();
    }

    void dynamicReconfigCallback(arm_controllers::PidParamsConfig &config, uint32_t /*level*/)
    {
        ROS_DEBUG_STREAM_NAMED("pid","Dynamics reconfigure callback recieved.");

        // Set the gains
        set(config.p, config.i, config.d);
    }

    double p_;
    double i_;
    double d_;
    double i_clamp_min_;
    double i_clamp_max_;
    bool antiwindup_;

private:
    // Store the PID gains in a realtime buffer to allow dynamic reconfigure to update it without
    // blocking the realtime update loop
    // realtime_tools::RealtimeBuffer<Pid> gains_buffer_;

    // Dynamics reconfigure
    bool dynamic_reconfig_initialized_;
    typedef dynamic_reconfigure::Server<arm_controllers::PidParamsConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;

    boost::recursive_mutex param_reconfig_mutex_;
};
}
#endif