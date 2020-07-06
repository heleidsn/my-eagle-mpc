#ifndef MULTICOPTER_MPC_MPC_MAIN_HPP_
#define MULTICOPTER_MPC_MPC_MAIN_HPP_

#define MOTOR_TH_NORM_MIN -1.0
#define MOTOR_TH_NORM_MAX 1.0

#define MOTOR_SPEED_MAX 838.0
#define MOTOR_SPEED_MIN 0.0

#include <string>

// #include <Eigen/Core>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/ocp/low-level-controller.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/ocp/trajectory-generator-controller.hpp"
#include "multicopter_mpc/path.h"

namespace multicopter_mpc {

struct MultiCopterTypes {
  enum Type { Iris, Hector, NbMultiCopterTypes };
};

class MpcMain {
 public:
  MpcMain();
  MpcMain(MultiCopterTypes::Type mc_type, SolverTypes::Type solver_type, std::string mission_name);
  ~MpcMain();

  const boost::shared_ptr<const TrajectoryGeneratorController> getLowLevelController();
  void setCurrentState(const Eigen::Ref<Eigen::VectorXd>& current_state);
  const Eigen::VectorXd& runMpcStep();

 private:
  void computeSpeedControls();

  MultiCopterTypes::Type mc_type_;
  SolverTypes::Type solver_type_;

  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<Mission> mission_;

  double dt_;

  // boost::shared_ptr<LowLevelController> low_level_controller_;
  boost::shared_ptr<TrajectoryGeneratorController> low_level_controller_;
  std::size_t low_level_controller_knots_;
  boost::shared_ptr<TrajectoryGenerator> trajectory_generator_;

  Eigen::VectorXd current_state_;
  Eigen::VectorXd next_state_;
  Eigen::VectorXd current_motor_thrust_;
  Eigen::VectorXd current_motor_speed_;
  Eigen::VectorXd next_control_;
  std::size_t trajectory_cursor_;
};

}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MPC_MAIN_HPP_