/*
Author(s): Djordje Vukcevic, Sven Schneider
Copyright (c) [2017]

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

#include <solver_vereshchagin.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <boost/assign/list_of.hpp>
#include <stdlib.h>     /* abs */


// const int NUMBER_OF_JOINTS = 2;
// const int NUMBER_OF_SEGMENTS = 2;
const int NUMBER_OF_CONSTRAINTS = 6;

class extended_kinematic_chain
{
    public:
        KDL::Chain chain;
        std::vector<double> joint_inertia;
        std::vector<double> joint_static_friction;
};

class motion_specification
{
    public:
        motion_specification(
                int number_of_joints,
                int number_of_segments,
                int number_of_constraints)
            : q(number_of_joints),
              qd(number_of_joints),
              qdd(number_of_joints),
              feedforward_torque(number_of_joints),  //Initial Q without friction
              end_effector_unit_constraint_forces(number_of_constraints), //alpha
              end_effector_acceleration_energy_setpoint(number_of_constraints), //beta
              external_force(number_of_segments) //U in Vereshchagin 1989 paper
              {

              }

        KDL::JntArray q;
        KDL::JntArray qd;
        KDL::JntArray qdd; // as input to original solver, overwritten during call!
        KDL::JntArray feedforward_torque;
        KDL::Jacobian end_effector_unit_constraint_forces;
        KDL::JntArray end_effector_acceleration_energy_setpoint;
        KDL::Wrenches external_force;
};

void create_my_LWR_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 7;

    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.5;

    c.joint_static_friction.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 40.0;

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
}

void create_my_2DOF_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 2;
    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.1;

    c.joint_static_friction.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 40.0;
    // c.joint_static_friction[0] = 500;
    // c.joint_static_friction[1] = 5;

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
    // std::cout << c.chain.getNrOfJoints() << '\n';
    // std::cout << c.chain.getNrOfSegments() << '\n';
}

void create_my_motion_specification(motion_specification &m)
{
    m.q(0) = M_PI / 2.0;
    m.q(1) = M_PI / 6.0;

    m.qd(0) = 0.0;
    m.qd(1) = 0.0;

    m.feedforward_torque(0) = 0.0;
    m.feedforward_torque(1) = 0.0;

    m.external_force[0] = KDL::Wrench();
    m.external_force[1] = KDL::Wrench();

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(1.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 1.0;

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

class vereshchagin_with_friction {

    public:
        vereshchagin_with_friction(
                const extended_kinematic_chain &chain,
                const KDL::Twist &acc_root,
                int number_of_constraints)
            : solver_(chain.chain, acc_root, number_of_constraints),
              chain_(chain),
              tau_(chain.chain.getNrOfJoints()),
              qdd_(chain.chain.getNrOfJoints()),
              friction_torque_(chain.chain.getNrOfJoints())
        {
            number_of_frames_ = chain.chain.getNrOfSegments() + 1;
            number_of_joints_ = chain_.chain.getNrOfJoints();
            frame_acceleration_.resize(number_of_frames_);
            articulated_body_inertia_.resize(number_of_frames_);
            bias_force_.resize(number_of_frames_);
            max_acc_energy = 0;
        }

        void solve(motion_specification &m)
        {
            // current implementation of assert is for two joints only!
            // How this should be changed if the class is moved in separate file?
            // assert(number_of_joints_ == 7);

            const int NUMBER_OF_STEPS = 50;

            KDL::JntArray initial_friction (number_of_joints_);

            for (int i = 0; i < number_of_joints_; i++) initial_friction(i) = chain_.joint_static_friction[i];
            std::vector<double> resulting_torque_set(number_of_joints_);

            //Define resolution(step value) for each joint
            std::vector<double> resolution;
            for (int i = 0; i < number_of_joints_; i++){
                resolution.push_back((2 * initial_friction(i)) / (1.0 * NUMBER_OF_STEPS));
            }

            plot_file.open ("/home/djole/Downloads/Master/R_&_D/KDL_GIT/Testing_repo/src/Simulation/plot_data.txt");

            iterate_over_torques(m, initial_friction, resolution, 0, NUMBER_OF_STEPS, resulting_torque_set);

            for (int i = 0; i < number_of_joints_ + 1; i++) {
                if(i != number_of_joints_) plot_file << optimum_torques[i] <<" ";
                else plot_file << max_acc_energy;
            }
            plot_file.close();
        }

    private:

        void iterate_over_torques(
            motion_specification &m,
            const KDL::JntArray initial_friction,
            const std::vector<double> resolution,
            int joint_index,
            const int steps,
            std::vector<double> resulting_set)
        {

            if (joint_index >= number_of_joints_) {

                for(int j = 0; j < number_of_joints_; j++){
                    friction_torque_(j) = resulting_set[j];
                }

                //TODO
                // select_non_moving_joints(*temp_friction_torques);
                KDL::Subtract(m.feedforward_torque, friction_torque_, tau_);
                qdd_ = m.qdd;

                int result = solver_.CartToJnt(
                             m.q, m.qd, qdd_, //qdd_ is overwritten by accual/resulting acceleration
                             m.end_effector_unit_constraint_forces,       // alpha
                             m.end_effector_acceleration_energy_setpoint, // beta
                             m.external_force,
                             tau_);  // tau_ is overwritten!
                     assert(result == 0);

                 solver_.get_link_acceleration(frame_acceleration_);
                 solver_.get_link_inertias(articulated_body_inertia_);
                 solver_.get_bias_force(bias_force_);

                 double acc_energy = compute_acc_energy(
                         frame_acceleration_, articulated_body_inertia_,
                         bias_force_, qdd_, tau_);

                //write data in the file
                for (int k = 0; k < number_of_joints_ + 1; k++){
                    if(k != number_of_joints_){
                        plot_file << resulting_set[k] <<" ";
                    }
                    else{
                        plot_file << acc_energy <<"\n";
                    }
                }

                if (acc_energy > max_acc_energy){
                    max_acc_energy = acc_energy;
                    optimum_torques = resulting_set;
                }

                return;
            }

            else {
               for (int i = 0; i <= steps; i++){
                   resulting_set[joint_index] = -initial_friction(joint_index) +  (resolution[joint_index] * i);
                   iterate_over_torques(m, initial_friction, resolution, joint_index + 1, steps, resulting_set);
               }
            }
        }

        //Calculate acceleration energy based on Gauss least constraint principle
        double compute_acc_energy(
                const std::vector<KDL::Twist> &xdd,
                const std::vector<KDL::ArticulatedBodyInertia> &h,
                const KDL::Wrenches &bias_force,
                const KDL::JntArray &qdd,
                const KDL::JntArray &tau)
        {

            double acc_energy_joint = 0.0;
            for (int i = 0; i < number_of_joints_; i++)  {
                acc_energy_joint += 0.5 * (qdd(i) * chain_.joint_inertia[i] * qdd(i));
                acc_energy_joint -= 0.5 * (tau(i) * qdd(i));
                // acc_energy_joint += qdd(i);
            }

            double acc_energy_segment = 0.0;
            for (int i = 1; i < number_of_frames_; i++) {
                acc_energy_segment += 0.5 * dot(xdd[i], h[i] * xdd[i]);
                acc_energy_segment += 0.5 * dot(bias_force[i], xdd[i]);
            }

            return acc_energy_joint + acc_energy_segment;
        }


        // void select_non_moving_joints(KDL::JntArray &temp_friction_torques){
        //
        //     for (int i = 0; i < num_joint; i++) {
        //         //TODO check for the right 0 treshold!!!!! Best ask Sven for float inacuracy
        //         //TODO test functioon by calling it with simulation
        //         //there is no sense calling it witout simulation
        //         //But it seams that works
        //         if (abs(init_params.jointRates(i)) > 0.01){
        //             temp_friction_torques(i) = 0;
        //         }
        //         else {
        //             std::cout <<"Not moving!!  "<< i << " " <<temp_friction_torques(i) << '\n';
        //         }
        //     }
        // }

        KDL::JntArray tau_; // as input to original solver (sum of external feed-forward torque and friction torque), overwritten during call!
        KDL::JntArray qdd_; // as input to original solver, overwritten during call!
        KDL::JntArray friction_torque_;
        KDL::Solver_Vereshchagin solver_;
        int number_of_frames_;
        int number_of_joints_;
        extended_kinematic_chain chain_;

        std::vector<KDL::Twist> frame_acceleration_;
        std::vector<KDL::ArticulatedBodyInertia> articulated_body_inertia_;
        KDL::Wrenches bias_force_;

        double max_acc_energy;
        std::vector<double> optimum_torques;

        std::ofstream plot_file;
};


int main(int argc, char* argv[])
{
    extended_kinematic_chain my_robot;
    // create_my_LWR_robot(my_robot);
    create_my_2DOF_robot(my_robot);
    motion_specification my_motion(my_robot.chain.getNrOfJoints(), my_robot.chain.getNrOfSegments(), NUMBER_OF_CONSTRAINTS);
    create_my_motion_specification(my_motion);

    //arm root acceleration
    KDL::Vector linearAcc(0.0, -9.81, 0.0); //gravitational acceleration along Y
    // KDL::Vector linearAcc(0.0, 0.0, - 9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    vereshchagin_with_friction solver(my_robot, root_acc, NUMBER_OF_CONSTRAINTS);
    solver.solve(my_motion);

	return 0;
}
