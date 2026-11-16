// 프로젝트 헤더: 클래스 선언과 멤버 변수는 .hpp에, 함소 본문은 .cpp에 구현
#include "joint_trajectory_command_broadcaster/joint_trajectory_commande_broadcaster.hpp"
// 그리퍼 판별 유효 목표 저장, 그리퍼 유지 보조 함수가 선언된 헤더
#include "joint_trajectory_command_broadcaster/leader_action_latch.hpp"

// 표준 라이브러리 헤더 묶음입니다. <> 안은 헤더 이름이며 함수 호출이 아닙니다
# include <chrono>
# include <cstddef>
# include <limits>
# include <memory>
# include <string>
# include <thread>
# include <unordered_map>
# include <vector>
# include <functional>
# include <cmath>
# include <algorithm>
# include <iterator>

// Eigen은 행렬 계산, KDL은 로봇 체인의 순기구학과 Jacobian 계산에 사용합니다.
#include <Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

// 전방 선언: State라는 클래스가 있다는 사실만 알립니다. 본문에는 다
namespace rclcpp_lifecycle
{
  class State;
}

namespace joint_trajectory_command_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
}

// 운동학 데이터 구조와 생성자
struct JointTrajectoryCommandBroadcaster::GroupPoseKinematics
{
    struct ChainKinematics
    {
    explicit ChainKinematics(const KDL::Chain & configured_chain)
    : chain(configured_chain),
      fk_solver(std::make_unique<KDL::ChainFkSolverPos_recursive>(chain)),
      jacobian_solver(std::make_unique<KDL::ChainJntToJacSolver>(chain))
      {
      }
      KDL::Chain chain;
      std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
      std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
      std::vector<size_t> group_joint_indices;
    };
    
    GroupPoseKinematics(
        const KDL::Chain & leader_chain, const KDL::Chain & follower_chain)
        : leader(leader_chain), follower(follower_chain)
    {
    }
    
    ChainKinematics leader;
    ChainKinematics follower;
    RelativePoseMapper relative_mapper;
};

// 이름 없는 namespace: 내부에서 쓰는 보조 함수를 둡니다
namespace
{

    template <typename KinematicsT>
    bool compute_pose(
        KinematicsT & kinematics,
        const std::vector<double> & group_position,
        // pose는 출력용 참조입니다. 함수 안에서 수정하면 호출자의 pose도 바뀝니다
        Eigen::Isometry3d & pose,
        // 선택적 출력 포인터입니다. nullptr이면 관절 배열 결과를 별도로 돌려주지 않습니다.
        KDL::JntArray * joint_array_out = nullptr)
        {
            // 계산 전에 관절 개수가 맞는 지 검사하는 조기 반환 패턴입니다.
            if (kinematics.chain.getNr0fJoints() != kinematics.chain.getNr0fJoints()) {
                return false;
            }
        }
        KDL::JntArray joints(kinematics.chain.getNr0fJoints());
        for (size_t i = 0; i < kinematics.chain.getNr0fJoints(); ++i) {
            const size_t group_index = kinematics.group_joint_indices[i];
            // 배열 범위를 먼저 확인하고, 값이 NaN/무한대인지 검사합니다
            if (group_index >= group_position.size() || !std::isfinite(group_position[group_index])) {
                return false;
            }
            joints(static_cast<unsigned int>(i)) = group_position[group_index];
        }

        KDL::Frame frame;
        if (kinematics.fk_solver->JntToCart(joint, frame) < 0){
            return false;
        }

        pose.setIdentity();
        for (size_t row = 0; row < 3; ++row){
            pose.translation()(static_cast<Eigen::Index>(row)) = frame.p[static_cast<unsigned int>(row)];
            for (size_t col = 0; col < 3; ++col){
                pose.linear()(static_cast<Eigen::Index>(row)) = frame.p[static_cast]
            }

