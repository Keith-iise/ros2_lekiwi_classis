#include "lekiwi/lekiwi_base.hpp"
#include <pluginlib/class_list_macros.hpp>
namespace lekiwi_controller
{
    LekiwiBase::LekiwiBase()
    : use_serial_(true), serial_baud_(1000000), torque_enabled_(true)
    {
    }
    LekiwiBase::~LekiwiBase()
    {
        if(use_serial_)
        {
            st3215_.end();
        }
    }
    CallbackReturn LekiwiBase::on_init(const hardware_interface::HardwareInfo& hardware_info)
    {
        // 调用父类初始化
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS) return result;

        // 读取YAML配置参数
        use_serial_ = hardware_info.hardware_parameters.count("use_serial") ?
        (hardware_info.hardware_parameters.at("use_serial") == "true") : false;

        serial_port_ = hardware_info.hardware_parameters.count("serial_port") ?
        hardware_info.hardware_parameters.at("serial_port") : "/dev/ttyACM0";

        serial_baud_ = hardware_info.hardware_parameters.count("serial_baud") ?
        std::stoi(hardware_info.hardware_parameters.at("serial_baud")) : 1000000;


        size_t num_joints = info_.joints.size();
        // 命令变量接口
        velocity_comads_.resize(num_joints, 0.0);
        // 状态变量接口
        velocity_states_.resize(num_joints, 0.0);
        
        RCLCPP_INFO(rclcpp::get_logger(NodeName), "初始化 Lekiwi 成功 , 关节数量: %zu 个", num_joints);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> LekiwiBase::export_state_interfaces()
    {
        // 创建状态接口
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // 遍历关节，定义状态接口为速度模式
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        }
        // 导出接口
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> LekiwiBase::export_command_interfaces()
    {
        // 创建命令接口
        std::vector<hardware_interface::CommandInterface> comad_interfaces;

        // 遍历关节，定义速度模式关节的命令控制接口
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            comad_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_comads_[i]);
        }
        
        // 导出接口
        return comad_interfaces;
    }

    CallbackReturn LekiwiBase::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger(NodeName), "正在尝试激活 Lekiwi 底层...");

        if (use_serial_)
        {
            if (!st3215_.begin(serial_baud_, serial_port_.c_str()))
            {
                RCLCPP_ERROR(rclcpp::get_logger(NodeName), "初始化电机失败!!");
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger(NodeName), "串口初始化成功： %s", serial_port_.c_str());
        }

        // node_ = rclcpp::Node::make_shared("lekiwi_driver");

        // executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        // executor_->add_node(node_);
        // spin_thread_ = std::thread([this]() { executor_->spin(); });
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            uint8_t servo_id = static_cast<uint8_t>(i + start_sevro_id_);
            st3215_.EnableTorque(servo_id, 1);  // 1 = 使能
            RCLCPP_INFO(rclcpp::get_logger(NodeName), "电机 %d 已使能", servo_id);
        }

        RCLCPP_INFO(rclcpp::get_logger(NodeName), "Lekiwi 底层激活成功！！");
        return CallbackReturn::SUCCESS;
    }

    ReturnType LekiwiBase::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    { 
        // 先把所有状态设为 0，防止出现 NaN！
        std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

        if(use_serial_)
        {
            for(size_t i = 0; i < info_.joints.size(); i++)
            {
                uint8_t servo_id = static_cast<uint8_t>(i + start_sevro_id_);
                // std::this_thread::sleep_for(std::chrono::milliseconds(1));

                if (st3215_.FeedBack(servo_id) != -1)
                {
                    int raw_speed = st3215_.ReadSpeed(servo_id);
                    double speed_deg_s = raw_speed * (360.0 / 4096.0);
                    velocity_states_[i] = speed_deg_s * (M_PI / 180.0) * servo_directions_[i];

                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(NodeName), "无法连接电机: %d", servo_id);
                }
            }

        }
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type LekiwiBase::write(const rclcpp::Time&, const rclcpp::Duration&)
    {


        if (!use_serial_) return hardware_interface::return_type::OK;

        for (size_t i = 0; i < info_.joints.size(); i++) {
            uint8_t servo_id = static_cast<uint8_t>(i + start_sevro_id_);
            double velocity_rad_s = velocity_comads_[i];

            double velocity_deg_s = velocity_rad_s * (180.0 / M_PI);
            double steps_per_deg = 4096.0 / 360.0;
            int16_t wheel_speed = static_cast<int16_t>(velocity_deg_s * steps_per_deg);

            wheel_speed *= servo_directions_[i];
            RCLCPP_DEBUG(rclcpp::get_logger(NodeName),
                            "WHEEL Servo %d velocity: %.3f rad/s -> %.1f deg/s -> %d raw", servo_id, velocity_rad_s,
                            velocity_deg_s, wheel_speed);
            if (!st3215_.WriteSpe(servo_id, wheel_speed, 50)){
                RCLCPP_WARN(rclcpp::get_logger(NodeName), "无法写入轮子电机: %d", servo_id);
            }
        }

        st3215_.RegWriteAction();
        return hardware_interface::return_type::OK;
    }


    CallbackReturn LekiwiBase::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        if (use_serial_)
        {
            for (size_t i = 0; i < info_.joints.size(); ++i)
            {
                uint8_t servo_id = static_cast<uint8_t>(i + start_sevro_id_);
                st3215_.EnableTorque(servo_id, 0);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger(NodeName), "Lekiwi 底层已停止");
        return hardware_interface::CallbackReturn::SUCCESS;

    }


} // namespace

PLUGINLIB_EXPORT_CLASS(lekiwi_controller::LekiwiBase, hardware_interface::SystemInterface)







