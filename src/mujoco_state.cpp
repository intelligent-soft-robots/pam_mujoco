#include "pam_mujoco/mujoco_state.hpp"


namespace pam_mujoco
{
    MujocoStatePrinter::MujocoStatePrinter(std::string filepath)
        : m_file_(filepath), m_thread_(&MujocoStatePrinter::write_to_file, this), m_exit_(false)
    {
        if (!m_file_)
            throw std::runtime_error("Could not open file");
        m_file_ << "time_stamp" << ","
                << "robot_position_0" << "," << "robot_position_1" << ","
                << "robot_position_2" << "," << "robot_position_3" << ","
                << "robot_velocity_0" << "," << "robot_velocity_1" << ","
                << "robot_velocity_2" << "," << "robot_velocity_3" << ","
                << "ball_position_0" << "," << "ball_position_1" << ","
                << "ball_position_2" << "," 
                << "ball_velocity_0" << "," << "ball_velocity_1" << ","
                << "ball_velocity_2" << "\n"; 

    }

    MujocoStatePrinter::~MujocoStatePrinter()
    {
        if (!m_exit_)
            {
                exit();
            }
    }
    
    void MujocoStatePrinter::add(const MujocoState& state)
    {
        std::unique_lock<std::mutex> lock(m_mutex_);
        if (m_exit_)
            return;
        m_queue_.push(state);
        m_cond_var_.notify_one();
    }

    void MujocoStatePrinter::exit()
    {
        {
            std::unique_lock<std::mutex> lock(m_mutex_);
            m_exit_ = true;
        }
        m_cond_var_.notify_one();
        m_thread_.join();
    }

    void MujocoStatePrinter::write_to_file()
    {
        while (true)
            {
                std::unique_lock<std::mutex> lock(m_mutex_);
                m_cond_var_.wait(lock, [this] { return !m_queue_.empty() || m_exit_; });
                
                if (m_exit_ && m_queue_.empty())
                    return;
                
                MujocoState state = m_queue_.front();
                m_queue_.pop();
                lock.unlock();

                m_file_ << std::fixed << std::setprecision(3) << state.time_stamp << ','
                       << state.robot_positions[0] << ',' << state.robot_positions[1] << ','
                       << state.robot_positions[2] << ',' << state.robot_positions[3] << ','
                       << state.robot_velocities[0] << ',' << state.robot_velocities[1] << ','
                       << state.robot_velocities[2] << ',' << state.robot_velocities[3] << ','
                       << state.ball_positions[0] << ',' << state.ball_positions[1] << ','
                       << state.ball_positions[2] << ','
                       << state.ball_velocities[0] << ',' << state.ball_velocities[1] << ','
                       << state.ball_velocities[2] << '\n';
            }
    }

    std::string format_filename(const std::string& folder, const std::string& filename)
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_c);
        
        std::ostringstream oss;
        oss << folder << '/' 
            << std::put_time(now_tm, "%Y_%m_%d_%H_%M_") 
            << filename << ".csv";
        
        return oss.str();
    }
    
    MujocoStatePrinterController::MujocoStatePrinterController(std::string mujoco_id,
                                                               std::string folder,
                                                               std::string robot_joint,
                                                               std::string ball_joint)
        : msp_{format_filename(folder, mujoco_id)},
          robot_joint_(robot_joint),
          qpos_robot_(-2),
          qvel_robot_(-2),
          ball_joint_(ball_joint),
          qpos_ball_(-2),
          qvel_ball_(-2)
    {}
          
    void MujocoStatePrinterController::init(const mjModel* m)
    {
        if (qpos_ball_ < -1)
            {
                if(ball_joint_!=std::string(""))
                    {
                        qpos_ball_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, ball_joint_.c_str())];
                        qvel_ball_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, ball_joint_.c_str())];
                    }
                else
                    {
                        qpos_ball_ = -1;
                        qvel_ball_ = -1;
                    }
                if(robot_joint_!=std::string(""))
                    {
                        qpos_robot_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, robot_joint_.c_str())];
                        qvel_robot_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, robot_joint_.c_str())];
                    }
                else
                    {
                        qpos_robot_ = -1;
                        qvel_robot_ = -1;
                    }
            }
    }

    double get_ts() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        double value = duration.count()*1e-9;
        return value;
    }
    
    void MujocoStatePrinterController::apply(const mjModel* m, mjData* d)
    {
        init(m);
        state_.time_stamp = get_ts();
        for (std::size_t dof = 0; dof < 4; dof++)
            {
                if(qpos_robot_!=-1)
                    {
                        state_.robot_positions[dof] = d->qpos[qpos_robot_+dof];
                        state_.robot_velocities[dof] = d->qvel[qvel_robot_+dof];
                    }
            }
        for (std::size_t dim = 0; dim < 3; dim++)
            {
                if(qpos_ball_!=-1)
                    {
                        state_.ball_positions[dim] = d->qpos[qpos_ball_+dim];
                        state_.ball_velocities[dim] = d->qvel[qvel_ball_+dim];
                    }
            }
        msp_.add(state_);
    }
    
}
