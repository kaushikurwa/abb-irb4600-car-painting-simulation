#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

std::string readFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    return buffer.str();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<rclcpp::Node>(
            "moveit_jacobian_validator");

        // Load the exact URDF and temporary SRDF.
        std::string urdf = readFile("/tmp/painting_cell.urdf");
        std::string srdf = readFile("/tmp/painting_cell_test.srdf");

        robot_model_loader::RobotModelLoader::Options options(
            urdf, srdf);

        options.load_kinematics_solvers_ = false;

        robot_model_loader::RobotModelLoader loader(node, options);

        auto model = loader.getModel();

        if (!model)
        {
            throw std::runtime_error("Failed to load robot model.");
        }

        std::cout << "\nRobot model loaded successfully.\n";
        std::cout << "Model frame: "
                  << model->getModelFrame() << "\n";

        // Retrieve the combined seven-joint group.
        const auto* group = model->getJointModelGroup("rail_arm");

        if (!group)
        {
            throw std::runtime_error(
                "Could not find rail_arm planning group.");
        }

        // Retrieve the TCP link.
        const auto* tcp = model->getLinkModel("paint_tcp");

        if (!tcp)
        {
            throw std::runtime_error(
                "Could not find paint_tcp link.");
        }

        // Create robot state.
        moveit::core::RobotState state(model);
        state.setToDefaultValues();

        // Set actual joint positions from your /joint_states output.
        state.setJointPositions(
            "rail_joint",
            std::vector<double>{-0.052167883669839724});

        state.setJointPositions(
            "joint_1",
            std::vector<double>{-0.0010874867762562272});

        state.setJointPositions(
            "joint_2",
            std::vector<double>{0.5194168230679344});

        state.setJointPositions(
            "joint_3",
            std::vector<double>{-0.3687419342689288});

        state.setJointPositions(
            "joint_4",
            std::vector<double>{0.0});

        state.setJointPositions(
            "joint_5",
            std::vector<double>{1.4201214379958238});

        state.setJointPositions(
            "joint_6",
            std::vector<double>{-1.571880140364659});

        state.update();

        // Print the actual MoveIt variable ordering.
        std::cout << "\nJoint variable ordering:\n";

        for (const auto& name : group->getVariableNames())
        {
            std::cout << name << "\n";
        }

        // TCP position in the robot model frame.
        Eigen::Isometry3d tcp_transform =
            state.getGlobalLinkTransform(tcp);

        std::cout << "\nTCP position in model frame:\n";
        std::cout << tcp_transform.translation().transpose()
                  << "\n";

        // Calculate the geometric Jacobian at the TCP origin.
        Eigen::MatrixXd jacobian;

        bool success = state.getJacobian(
            group,
            tcp,
            Eigen::Vector3d::Zero(),
            jacobian);

        if (!success)
        {
            throw std::runtime_error(
                "MoveIt failed to calculate the Jacobian.");
        }

        std::cout << "\nMoveIt Jacobian dimensions: "
                  << jacobian.rows() << " x "
                  << jacobian.cols() << "\n";

        std::cout << "\nMoveIt Jacobian:\n";
        std::cout << jacobian << "\n";

    }
    catch (const std::exception& e)
    {
        std::cerr << "\nERROR: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
