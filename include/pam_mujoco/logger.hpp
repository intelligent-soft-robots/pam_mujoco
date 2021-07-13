#pragma once

#include "time_series/multiprocess_time_series.hpp"
#include "o80/time.hpp"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

  class LogRobot
  {
  public:
    template<class Archive>
    void serialize(Archive &archive)
    {
      archive(positions,velocities);
    }
    std::array<double,4> positions;
    std::array<double,4> velocities;
  };

  class LogBall
  {
  public:
    template<class Archive>
    void serialize(Archive &archive)
    {
      archive(position,velocity);
    }
    std::array<double,3> position;
    std::array<double,3> velocity;
  };

  class LogItem
  {
  public:
    template<class Archive>
    void serialize(Archive &archive)
    {
      archive(time_stamp,robot,ball);
    }
    long int time_stamp;
    LogRobot robot;
    LogBall ball;
  };

  typedef time_series::MultiprocessTimeSeries<LogItem> Ts;

  /**
   * Write at each iteration
   * in a multiprocess time series the positions
   * and velocities of the ball and of the robot
   */
  class Logger: public ControllerBase
  {
  public:
    Logger(size_t time_series_size,
	   std::string segment_id,
	   std::string robot_joint_base,
	   std::string ball_joint,
	   int ball_index_qpos,
	   int ball_index_qvel);
    void apply(const mjModel* m, mjData* d);
  private:
    std::string segment_id_;
    Ts time_series_;
    std::string robot_joint_base_;
    int index_q_robot_;
    int index_qvel_robot_;
    std::string ball_joint_;
    int ball_index_qpos_;
    int ball_index_qvel_;
    LogItem item_;
  };

}


