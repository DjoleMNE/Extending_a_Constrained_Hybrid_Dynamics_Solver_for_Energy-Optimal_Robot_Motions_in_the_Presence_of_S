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


const int NUMBER_OF_JOINTS = 2;
const int NUMBER_OF_SEGMENTS = 2;
const int NUMBER_OF_CONSTRAINTS = 1;

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
              feedforward_torque(number_of_joints),
              end_effector_unit_constraint_forces(number_of_constraints),
              end_effector_acceleration_energy_setpoint(number_of_constraints),
              external_force(number_of_segments)
        {
        }


        KDL::JntArray q;
        KDL::JntArray qd;
        KDL::JntArray qdd;
        KDL::JntArray feedforward_torque;
        KDL::Jacobian end_effector_unit_constraint_forces;
        KDL::JntArray end_effector_acceleration_energy_setpoint;
        KDL::Wrenches external_force;
};


class extended_kinematic_chain
{
    public:
        KDL::Chain chain;
        std::vector<double> joint_inertia;
        std::vector<double> joint_static_friction;
};


void create_my_motion_specification(motion_specification &m)
{
    m.q(0) = M_PI_2;
    m.q(1) = M_PI_2 / 3.0;

    m.qd(0) = 0.0;
    m.qd(1) = 0.0;

    m.feedforward_torque(0) = 0.0;
    m.feedforward_torque(1) = 0.0;

    m.external_force[0] = KDL::Wrench();
    m.external_force[1] = KDL::Wrench();

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 0.0;
}


void create_my_robot(extended_kinematic_chain &c)
{
    c.joint_inertia.resize(NUMBER_OF_JOINTS);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) c.joint_inertia[i] = 0.1;

    c.joint_static_friction.resize(NUMBER_OF_JOINTS);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) c.joint_static_friction[i] = 1.0;

    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
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
            frame_acceleration_.resize(number_of_frames_);
            articulated_body_inertia_.resize(number_of_frames_);
            bias_force_.resize(number_of_frames_);
        }

        void solve(motion_specification &m)
        {
            // current implementation is for two joints only!
            assert(chain_.chain.getNrOfJoints() == 2);

            const int NUMBER_OF_STEPS = 40;

            double fr0 = chain_.joint_static_friction[0];
            double fr1 = chain_.joint_static_friction[1];
            double res0 = (2.0 * fr0) / (1.0 * NUMBER_OF_STEPS);
            double res1 = (2.0 * fr1) / (1.0 * NUMBER_OF_STEPS);

            plot_file.open ("plot_data.txt");

            for (friction_torque_(0) = -fr0; friction_torque_(0) < fr0; friction_torque_(0) += res0) {
                for (friction_torque_(1) = -fr1; friction_torque_(1) < fr1; friction_torque_(1) += res1) {
                    KDL::Add(m.feedforward_torque, friction_torque_, tau_);
                    qdd_ = m.qdd;

                    int result = solver_.CartToJnt(
                            m.q, m.qd, qdd_,
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

                    plot_file << friction_torque_(0) << " " << friction_torque_(1) << " " << acc_energy << std::endl;
                }
            }

            plot_file.close();
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

            //check if here should be ff_torques+friction_torque of both!
            //From the paper it seams that Q is ff_torque + friction_torque
            double acc_energy_joint = 0.0;
            for (int i = 0; i < chain_.chain.getNrOfJoints(); i++)  {
                acc_energy_joint += 0.5 * (qdd(i) * chain_.joint_inertia[i] * qdd(i));
                acc_energy_joint -= 0.5 * (tau(i) * qdd(i));
            }

            double acc_energy_segment = 0.0;
            for (int i = 1; i < number_of_frames_; i++) {
                acc_energy_segment += 0.5 * dot(xdd[i], h[i] * xdd[i]);
                acc_energy_segment += 0.5 * dot(bias_force[i], xdd[i]);
            }

            return acc_energy_joint + acc_energy_segment;
        }


        KDL::JntArray tau_; // as input to original solver (sum of external feed-forward torque and friction torque), overwritten during call!
        KDL::JntArray qdd_; // as input to original solver, overwritten during call!
        KDL::JntArray friction_torque_;
        KDL::Solver_Vereshchagin solver_;
        int number_of_frames_;
        extended_kinematic_chain chain_;

        std::vector<KDL::Twist> frame_acceleration_;
        std::vector<KDL::ArticulatedBodyInertia> articulated_body_inertia_;
        KDL::Wrenches bias_force_;

        std::ofstream plot_file;
};



int main(int argc, char* argv[])
{
    extended_kinematic_chain my_robot;
    create_my_robot(my_robot);

    motion_specification my_motion(NUMBER_OF_JOINTS, NUMBER_OF_SEGMENTS, NUMBER_OF_CONSTRAINTS);
    create_my_motion_specification(my_motion);

    //arm root acceleration
    KDL::Vector linearAcc(0.0, -9.81, 0.0); //gravitational acceleration along Y
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    vereshchagin_with_friction solver(my_robot, root_acc, NUMBER_OF_CONSTRAINTS);
    solver.solve(my_motion);

	return 0;
}
