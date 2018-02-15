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


const int NUMBER_OF_CONSTRAINTS = 1;

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


void create_my_1DOF_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 1;
    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.0;

    c.joint_static_friction.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 0.0;

    for (int i = 0; i < number_of_joints; i++) {

        //last 3 inputs...input_scale, offset and joint inertia (d in paper)
        KDL::Joint joint = KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[i]);

        // RPY(roll,pitch,yaw) Rotation built from Roll-Pitch-Yaw angles
        KDL::Frame tip(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 1.0, 0.0));

        //Frames desctibe pose of the segment tip, wrt joint frame
        KDL::Segment segment = KDL::Segment(joint, tip);

        //rotational inertia around symmetry axis of rotation
        KDL::RotationalInertia rotational_inertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        //spatial inertia
        // center of mass at the same position as tip  of segment???
        KDL::RigidBodyInertia inertia(1, KDL::Vector(0.0, 1.0, 0.0), rotational_inertia);

        segment.setInertia(inertia);

        //adding segments in chain
        c.chain.addSegment(segment);
    }

}

void create_my_motion_specification(motion_specification &m)
{
    m.q(0) = M_PI / 2;
    //m.q(1) = 0.0;

    m.external_force[0] = KDL::Wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
    //m.external_force[1] = KDL::Wrench(KDL::Vector(0.0, 5.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 0.0;

}

class vereshchagin_with_friction {

    public:
        vereshchagin_with_friction(
                const extended_kinematic_chain &chain,
                const KDL::Twist &acc_root,
                int number_of_constraints)
            : solver_(chain.chain, acc_root, number_of_constraints), chain_(chain)
        {
            number_of_frames_ = chain.chain.getNrOfSegments() + 1;
            number_of_joints_ = chain_.chain.getNrOfJoints();
        }

        void solve(motion_specification &m)
        {
            std::vector<KDL::Twist> frame_acceleration_(number_of_frames_);
            std::vector<KDL::ArticulatedBodyInertia> articulated_body_inertia_(number_of_frames_);
            KDL::Wrenches bias_force_(number_of_frames_);
            KDL::JntArray tau_(number_of_joints_);
            KDL::JntArray qdd_of_mu(number_of_joints_);
            KDL::JntArray qdd_of_mu_star(number_of_joints_);

            double max_acc_energy = -1000.0;
            KDL::JntArray mu_star(number_of_joints_);

            for (double mu0 = -10.0; mu0 <= 10.0; mu0 += 0.5) {
                //for (double mu1 = -10.0; mu1 <= 10.0; mu1 += 0.5) {
                    tau_(0) = -mu0;
                    //tau_(1) = -mu1;
                    //qdd_of_mu(0) = qdd;

                    int result = solver_.CartToJnt(
                                 m.q, m.qd, qdd_of_mu, //qdd_of_mu is overwritten by accual/resulting acceleration
                                 m.end_effector_unit_constraint_forces,       // alpha
                                 m.end_effector_acceleration_energy_setpoint, // beta
                                 m.external_force,
                                 tau_);  // tau_ is overwritten!
                    assert(result == 0);

                    solver_.get_link_acceleration(frame_acceleration_);
                    solver_.get_link_inertias(articulated_body_inertia_);
                    solver_.get_bias_force(bias_force_);

                    tau_(0) = -mu0;
                    //tau_(1) = -mu1;

                    double z_of_qdd_of_mu = compute_acc_energy(
                             frame_acceleration_, articulated_body_inertia_,
                             bias_force_, qdd_of_mu, tau_);

                    if (z_of_qdd_of_mu > max_acc_energy) {
                        max_acc_energy = z_of_qdd_of_mu;
                        mu_star = tau_;
                        qdd_of_mu_star = qdd_of_mu;
                    }

                    std::cout << tau_(0) << " " << z_of_qdd_of_mu << std::endl;
                //}
            }
            //std::cout << mu_star(0) << " " << mu_star(1) << " " << max_acc_energy << std::endl;
        }

    private:

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
                acc_energy_joint -= (tau(i) * qdd(i));                                // qdd depends on tau, thus this is actually quadratic in tau, i.e. ~tau^2
            }

            double acc_energy_segment = 0.0;
            for (int i = 1; i < number_of_frames_; i++) {
                acc_energy_segment += 0.5 * dot(xdd[i], h[i] * xdd[i]);
                acc_energy_segment += dot(bias_force[i], xdd[i]);
            }

            return acc_energy_joint + acc_energy_segment;
        }

        KDL::Solver_Vereshchagin solver_;
        int number_of_frames_;
        int number_of_joints_;
        extended_kinematic_chain chain_;
};


int main(int argc, char* argv[])
{
    extended_kinematic_chain my_robot;
    create_my_1DOF_robot(my_robot);
    motion_specification my_motion(my_robot.chain.getNrOfJoints(), my_robot.chain.getNrOfSegments(), NUMBER_OF_CONSTRAINTS);
    create_my_motion_specification(my_motion);

    //KDL::Vector linearAcc(0.0, -9.81, 0.0); //gravitational acceleration along Y
    KDL::Vector linearAcc(0.0, 0.0, 0.0);
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    vereshchagin_with_friction solver(my_robot, root_acc, NUMBER_OF_CONSTRAINTS);
    solver.solve(my_motion);

	return 0;
}
