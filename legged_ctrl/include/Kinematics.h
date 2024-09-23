#include "RobotState.h"

namespace robot {
class Kinematics {
   public:
    Kinematics(){};

    static Eigen::Matrix<double, 3, 1> cal_left_ankle_pos_body(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, 1> cal_right_ankle_pos_body(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);

    static Eigen::Matrix<double, 3, 1> cal_left_toe_pos_body(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, 1> cal_left_heel_pos_body(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, 1> cal_right_toe_pos_body(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, 1> cal_right_heel_pos_body(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);

    static Eigen::Matrix<double, 3, LEG_DOF> cal_left_ankle_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, LEG_DOF> cal_right_ankle_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);

    static Eigen::Matrix<double, 3, LEG_DOF> cal_left_toe_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, LEG_DOF> cal_left_heel_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, LEG_DOF> cal_right_toe_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, LEG_DOF> cal_right_heel_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
};
}  // namespace robot
