/*
Author(s): Djordje Vukcevic, Sven Schneider
Copyright (c) [2018]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <vereshchagin_with_friction.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <stdlib.h>     /* abs */
#include <chrono>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
using nanos = std::chrono::microseconds;
using get_time = std::chrono::steady_clock;

// const int NUMBER_OF_JOINTS = 2;
// const int NUMBER_OF_SEGMENTS = 2;
const int NUMBER_OF_CONSTRAINTS = 6;

int sign_of(double x){
    if(x < 0.0){
        return -1;
    }
    else return 1;
}

int create_my_5DOF_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 5;

    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.3;

    c.joint_static_friction.resize(number_of_joints);
    // for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 5.0;
    c.joint_static_friction = {1.260, 0.956, 0.486, 0.300, 0.177};

    //Extract KDL tree from URDF file
    KDL::Tree yb_tree;
    urdf::Model yb_model;

    if (!yb_model.initFile("../urdf/youbot_arm_only.urdf")){
        std::cout << "ERROR: Failed to parse urdf robot model" << '\n';
        return 0;
    }

    if (!kdl_parser::treeFromUrdfModel(yb_model, yb_tree)){
        std::cout << "ERROR: Failed to construct kdl tree" << '\n';
        return 0;
    }

    //Extract KDL chain from KDL tree
    yb_tree.getChain("arm_link_0", "arm_link_5", c.chain);

    //In total 5 joints (NOT counting fixed - 0), 5 segments (NOT counting base link 0) and 6 frames
    return 1;
}

void create_my_LWR_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 7;

    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.5;

    c.joint_static_friction.resize(number_of_joints);
    // for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 10.0;
    c.joint_static_friction = {2.52,  1.912,  0.972,  0.6,  0.354,  0.354,  0.177};

    //Frames describe pose of the segment(base link) 0 tip, wrt joint 0 frame (inertial frame) - frame 0
    //Frame defenes pose of joint 1 in respect to joint 0 (inertial frame)
    //joint 0
	// c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
	// 			  KDL::Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)));

	//joint 1
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[0]),
				  KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
				  KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
										 KDL::Vector::Zero(),
										 KDL::RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

	//joint 2
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[1]),
				  KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
				  KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*KDL::RigidBodyInertia(2,
										   KDL::Vector(0.0,-0.3120511,-0.0038871),
										   KDL::RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

	//joint 3
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[2]),
				  KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
				  KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
										   KDL::Vector(0.0,-0.0015515,0.0),
										   KDL::RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

	//joint 4
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[3]),
				  KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
				  KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*KDL::RigidBodyInertia(2,
										   KDL::Vector(0.0,0.5216809,0.0),
										   KDL::RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

	//joint 5
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[4]),
				  KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
				  KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
										   KDL::Vector(0.0,0.0119891,0.0),
										   KDL::RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

	//joint 6
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[5]),
				  KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
				  KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
										   KDL::Vector(0.0,0.0080787,0.0),
										   KDL::RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));

    //Frame 8 - end-effector (link 8) frame - at same pose as joint 7, frame == indentity!
	//joint 7
	c.chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[6]),
				   KDL::Frame::Identity(),
				   KDL::RigidBodyInertia(2,
    									   KDL::Vector::Zero(),
    									   KDL::RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));
    //In total 8 joints (counting fixed - 0), 8 segments (counting base link 0) and 9 frames
    //In total 7 joints (NOT counting fixed - 0), 7 segments (NOT counting base link 0) and 8 frames
}

void create_my_2DOF_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 2;
    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.1;

    c.joint_static_friction.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 40.0;

    //Here joint 0 is moving - no fixed joints !
    //2 joints, 2 segments
    for (int i = 0; i < number_of_joints; i++) {

        //last 3 inputs...input_scale, offset and joint inertia (d in paper)
        KDL::Joint joint = KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[i]);

        // RPY(roll,pitch,yaw) Rotation built from Roll-Pitch-Yaw angles
        KDL::Frame tip(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.4, 0.0));

        //Frames desctibe pose of the segment tip, wrt joint frame
        KDL::Segment segment = KDL::Segment(joint, tip);

        //rotational inertia around symmetry axis of rotation
        KDL::RotationalInertia rotational_inertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        //spatial inertia
        // center of mass at the same position as tip  of segment???
        KDL::RigidBodyInertia inertia(0.3, KDL::Vector(0.0, 0.4, 0.0), rotational_inertia);

        segment.setInertia(inertia);

        //adding segments in chain
        c.chain.addSegment(segment);
    }
}

void create_F_ext_motion_specification(motion_specification &m)
{
    int number_of_joints = m.q.rows();
    int number_of_segments = m.external_force.size();

    m.q(0) = M_PI / 2.0;
    m.q(1) = M_PI / 6.0;
    // m.q(0) = 0.0;
    // m.q(1) = 0.0;

    for (int i = 2; i < number_of_joints; i++) m.q(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qdd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.feedforward_torque(i) = 0.0;

    // //external forces on the arm
    for (int i = 0; i < number_of_segments - 1; i++) {
        KDL::Wrench externalForce(
            KDL::Vector(0.0, 0.0, 0.0), //Force
            KDL::Vector(0.0, 0.0, 0.0)); //Torque
        m.external_force[i] = externalForce;
    }

    KDL::Wrench externalForceEE(
        KDL::Vector(-1.0, -1.0, 0.0), //Force
        KDL::Vector(0.0, 0.0, 0.0));//Torque
    m.external_force[number_of_segments - 1] = externalForceEE;

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 0.0;

    KDL::Twist unit_constraint_force_y(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_y);
    m.end_effector_acceleration_energy_setpoint(1) = 0.0;
    KDL::Twist unit_constraint_force_z(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_z);
    m.end_effector_acceleration_energy_setpoint(2) = 0.0;

    KDL::Twist unit_constraint_force_x1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_x1);
    m.end_effector_acceleration_energy_setpoint(3) = 0.0;
    KDL::Twist unit_constraint_force_y1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_y1);
    m.end_effector_acceleration_energy_setpoint(4) = 0.0;
    KDL::Twist unit_constraint_force_z1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_z1);
    m.end_effector_acceleration_energy_setpoint(5) = 0.0;
}

void create_ff_tau_motion_specification(motion_specification &m)
{
    int number_of_joints = m.q.rows();
    int number_of_segments = m.external_force.size();

    m.q(0) = M_PI / 2.0;
    m.q(1) = M_PI / 6.0;
    // m.q(0) = 0.0;
    // m.q(1) = 0.0;

    for (int i = 2; i < number_of_joints; i++) m.q(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qdd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.feedforward_torque(i) = 5.0;

    // //external forces on the arm
    for (int i = 0; i < number_of_segments - 1; i++) {
        KDL::Wrench externalForce(
            KDL::Vector(0.0, 0.0, 0.0), //Force
            KDL::Vector(0.0, 0.0, 0.0)); //Torque
        m.external_force[i] = externalForce;
    }

    KDL::Wrench externalForceEE(
        KDL::Vector(0.0, 0.0, 0.0), //Force
        KDL::Vector(0.0, 0.0, 0.0));//Torque
    m.external_force[number_of_segments - 1] = externalForceEE;

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 0.0;

    KDL::Twist unit_constraint_force_y(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_y);
    m.end_effector_acceleration_energy_setpoint(1) = 0.0;
    KDL::Twist unit_constraint_force_z(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_z);
    m.end_effector_acceleration_energy_setpoint(2) = 0.0;

    KDL::Twist unit_constraint_force_x1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_x1);
    m.end_effector_acceleration_energy_setpoint(3) = 0.0;
    KDL::Twist unit_constraint_force_y1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_y1);
    m.end_effector_acceleration_energy_setpoint(4) = 0.0;
    KDL::Twist unit_constraint_force_z1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_z1);
    m.end_effector_acceleration_energy_setpoint(5) = 0.0;
}

void create_constrained_motion_specification(motion_specification &m)
{
    int number_of_joints = m.q.rows();
    int number_of_segments = m.external_force.size();

    m.q(0) = M_PI / 2.0;
    m.q(1) = M_PI / 6.0;
    // m.q(0) = 0.0;
    // m.q(1) = 0.0;

    for (int i = 2; i < number_of_joints; i++) m.q(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qdd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.feedforward_torque(i) = 0.0;

    // //external forces on the arm
    for (int i = 0; i < number_of_segments - 1; i++) {
        KDL::Wrench externalForce(
            KDL::Vector(0.0, 0.0, 0.0), //Force
            KDL::Vector(0.0, 0.0, 0.0)); //Torque
        m.external_force[i] = externalForce;
    }

    KDL::Wrench externalForceEE(
        KDL::Vector(0.0, 0.0, 0.0), //Force
        KDL::Vector(0.0, 0.0, 0.0));//Torque
    m.external_force[number_of_segments - 1] = externalForceEE;

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(1.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 5.0;

    KDL::Twist unit_constraint_force_y(
            KDL::Vector(0.0, 1.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_y);
    m.end_effector_acceleration_energy_setpoint(1) = 0.0;

    KDL::Twist unit_constraint_force_z(
            KDL::Vector(0.0, 0.0, 1.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_z);
    m.end_effector_acceleration_energy_setpoint(2) = 0.0;

    KDL::Twist unit_constraint_force_x1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(1.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_x1);
    m.end_effector_acceleration_energy_setpoint(3) = 0.0;

    KDL::Twist unit_constraint_force_y1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 1.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_y1);
    m.end_effector_acceleration_energy_setpoint(4) = 0.0;

    KDL::Twist unit_constraint_force_z1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 1.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_z1);
    m.end_effector_acceleration_energy_setpoint(5) = 0.0;
}


void test_3_solvers(extended_kinematic_chain &my_robot, motion_specification &motion, bool total_effort = false)
{
    create_F_ext_motion_specification(motion);
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z of arm root
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    // vereshchagin_with_friction extended_solver(my_robot, root_acc, NUMBER_OF_CONSTRAINTS);
    // auto start = get_time::now(); //use auto keyword to minimize typing strokes
    // extended_solver.solve(motion);
    // auto end = get_time::now();
    // auto diff = end - start;
    //
    // std::cout << "Extended Vereshchagin solver" << '\n';
    // std::cout << "Joint torques:        "<<extended_solver.true_control_torques << '\n';
    // std::cout << " " << '\n';
    // std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff).count()<<" microseconds "<<std::endl;
    // std::cout << " " << '\n'<<std::endl;

    KDL::Solver_Vereshchagin ver_solver(
                            my_robot.chain,
                            KDL::Twist(
                                KDL::Vector(0.0, 0.0, -9.81),
                                KDL::Vector(0.0, 0.0, 0.0)),
                            NUMBER_OF_CONSTRAINTS);

    KDL::JntArray control_torque_Ver(my_robot.chain.getNrOfJoints());
    create_F_ext_motion_specification(motion);

    auto start2 = get_time::now();
    int result2 = ver_solver.CartToJnt(
                 motion.q, motion.qd, motion.qdd,
                 motion.end_effector_unit_constraint_forces,       // alpha
                 motion.end_effector_acceleration_energy_setpoint, // beta
                 motion.external_force,
                 motion.feedforward_torque); // without friction
    auto end2 = get_time::now();
    assert(result2 == 0);
    auto diff2 = end2 - start2;

    ver_solver.get_control_torque(control_torque_Ver);
    std::cout << "Original Vereshchagin solver" << '\n';
    std::cout << "Joint torques:        "<<control_torque_Ver << '\n';
    if(total_effort){
        std::vector<double> full_effort(my_robot.chain.getNrOfJoints());
        std::cout << "Total effort:           ";
        for(int i = 0; i < my_robot.chain.getNrOfJoints(); i++){
            full_effort[i] = control_torque_Ver(i) + my_robot.joint_static_friction[i] * sign_of(control_torque_Ver(i));
            std::cout <<full_effort[i] << "    ";
        }
        std::cout << " " << '\n';
    }
    
    std::cout << " " << '\n';
    std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff2).count()<<" microseconds "<<std::endl;
    std::cout << " " << '\n'<<std::endl;
    // Eigen::VectorXd nu(NUMBER_OF_CONSTRAINTS, 1);
    // ver_solver.get_constraint_magnitude(nu);
    // std::cout << nu << '\n';

    // KDL::ChainIdSolver_RNE RNE_idsolver(my_robot.chain, KDL::Vector(0.0, 0.0, -9.81));
    // KDL::JntArray control_torque_RNE(my_robot.chain.getNrOfJoints());
    //
    // create_F_ext_motion_specification(motion);
    //
    // auto start3 = get_time::now();
    // int result3 = RNE_idsolver.CartToJnt(
    //                     motion.q,
    //                     motion.qd,
    //                     motion.qdd,
    //                     motion.external_force,
    //                     control_torque_RNE);
    // assert(result3 == 0);
    // auto end3 = get_time::now();
    // auto diff3 = end3 - start3;
    // std::cout << "Recursive Newton Euler solver" << '\n';
    // std::cout << "Joint torques:        "<<control_torque_RNE << '\n';
    // if(total_effort){
    //     std::vector<double> full_effort(my_robot.chain.getNrOfJoints());
    //     std::cout << "Total effort:           ";
    //     for(int i = 0; i < my_robot.chain.getNrOfJoints(); i++){
    //         full_effort[i] = control_torque_RNE(i) + my_robot.joint_static_friction[i] * sign_of(control_torque_RNE(i));
    //         std::cout <<full_effort[i] << "    ";
    //     }
    //     std::cout << "\n";
    // }
    //
    // std::cout <<'\n';
    // std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff3).count()<<" microseconds "<<std::endl;
    // std::cout <<'\n';
}

void test_2_models_FD(extended_kinematic_chain &robot_1, extended_kinematic_chain &robot_2, motion_specification &motion_1, motion_specification &motion_2)
{
    create_ff_tau_motion_specification(motion_1);
    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    vereshchagin_with_friction extended_solver_1(robot_1, root_acc, NUMBER_OF_CONSTRAINTS);
    auto start = get_time::now(); //use auto keyword to minimize typing strokes
    extended_solver_1.solve(motion_1);
    auto end = get_time::now();
    auto diff = end - start;

    std::cout << "Robot 1" << '\n';
    std::cout << "Joint accelerations:  "<<extended_solver_1.true_joint_acc << '\n';
    std::cout << " " << '\n';
    std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff).count()<<" microseconds "<<std::endl;
    std::cout << " " << '\n';

    create_ff_tau_motion_specification(motion_2);

    vereshchagin_with_friction extended_solver_2(robot_2, root_acc, NUMBER_OF_CONSTRAINTS);
    auto start2 = get_time::now(); //use auto keyword to minimize typing strokes
    extended_solver_2.solve(motion_2);
    auto end2 = get_time::now();
    auto diff2 = end2 - start2;

    std::cout << "Robot 2" << '\n';
    std::cout << "Joint accelerations:  "<<extended_solver_2.true_joint_acc << '\n';
    std::cout << " " << '\n';
    std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff2).count()<<" microseconds "<<std::endl;
    std::cout << " " << '\n';
}

void test_2_models_ID(extended_kinematic_chain &robot_1, extended_kinematic_chain &robot_2, motion_specification &motion_1, motion_specification &motion_2)
{
    create_constrained_motion_specification(motion_1);
    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    vereshchagin_with_friction extended_solver_1(robot_1, root_acc, NUMBER_OF_CONSTRAINTS);
    auto start = get_time::now(); //use auto keyword to minimize typing strokes
    extended_solver_1.solve(motion_1);
    auto end = get_time::now();
    auto diff = end - start;

    std::cout << "Robot 1" << '\n';
    std::cout << "Joint torques:        "<<extended_solver_1.true_control_torques << '\n';
    std::cout << " " << '\n';
    std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff).count()<<" microseconds "<<std::endl;
    std::cout << " " << '\n';

    create_constrained_motion_specification(motion_2);

    vereshchagin_with_friction extended_solver_2(robot_2, root_acc, NUMBER_OF_CONSTRAINTS);
    auto start2 = get_time::now(); //use auto keyword to minimize typing strokes
    extended_solver_2.solve(motion_2);
    auto end2 = get_time::now();
    auto diff2 = end2 - start2;

    std::cout << "Robot 2" << '\n';
    std::cout << "Joint torques:        "<<extended_solver_2.true_control_torques << '\n';
    std::cout << " " << '\n';
    std::cout<<"Elapsed time:  "<< std::chrono::duration_cast<nanos>(diff2).count()<<" microseconds "<<std::endl;
    std::cout << " " << '\n';
}

int main(int argc, char* argv[])
{
    //extended_kinematic_chain my_robot;
    // create_my_2DOF_robot(my_robot);

    extended_kinematic_chain five_DOF_robot;
    int result = create_my_5DOF_robot(five_DOF_robot);
    assert(result == 1);
    motion_specification five_DOF_motion(five_DOF_robot.chain.getNrOfJoints(), five_DOF_robot.chain.getNrOfSegments(), NUMBER_OF_CONSTRAINTS);

    extended_kinematic_chain LWR_robot;
    create_my_LWR_robot(LWR_robot);
    motion_specification LWR_motion(LWR_robot.chain.getNrOfJoints(), LWR_robot.chain.getNrOfSegments(), NUMBER_OF_CONSTRAINTS);

    // std::cout << "Testing 5DOF robot model with 3 different solvers" << '\n'<<std::endl;
    // std::cout << "5DOF robot friction torques:        ";
    // for(int i=0; i<five_DOF_robot.chain.getNrOfJoints(); ++i)  std::cout << five_DOF_robot.joint_static_friction[i] << "  ";
    // std::cout <<'\n'<<std::endl;
    // test_3_solvers(five_DOF_robot, five_DOF_motion, true);
    // std::cout << " " << '\n'<<std::endl;

    std::cout << "Testing LWR robot model with 3 different solvers" << '\n'<<std::endl;
    std::cout << "LWR friction torques:        ";
    for(int i=0; i<LWR_robot.chain.getNrOfJoints(); ++i)  std::cout << LWR_robot.joint_static_friction[i] << "  ";
    std::cout <<'\n'<<std::endl;
    test_3_solvers(LWR_robot, LWR_motion, false);
    //
    // std::cout << " " << '\n'<<std::endl;
    //
    // std::cout << "Testing(FD) 2 robot models with the same extended Vereshchagin solver" << '\n'<<std::endl;
    // test_2_models_FD(five_DOF_robot, LWR_robot, five_DOF_motion, LWR_motion);
    //
    // std::cout << " " << '\n'<<std::endl;
    //
    // std::cout << "Testing(ID) 2 robot models with the same extended Vereshchagin solver" << '\n'<<std::endl;
    // test_2_models_ID(five_DOF_robot, LWR_robot, five_DOF_motion, LWR_motion);

	return 0;
}
