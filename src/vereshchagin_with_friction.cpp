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
#include <extended_kinematic_chain.hpp>
#include <motion_specification.hpp>
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

vereshchagin_with_friction::vereshchagin_with_friction(
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
        transformed_frame_acceleration_.resize(number_of_frames_);
        true_frame_acceleration_.resize(number_of_frames_);
	    articulated_body_inertia_.resize(number_of_frames_);
	    bias_force_.resize(number_of_frames_);
	    control_tau_.resize(number_of_joints_);
	    true_control_torques.resize(number_of_joints_);
	    true_joint_acc.resize(number_of_joints_);
	    max_acc_energy = 0;
	}

void vereshchagin_with_friction::solve(motion_specification &m){

    assert(number_of_joints_ == m.q.rows());

    const int NUMBER_OF_STEPS = 2;

    KDL::JntArray initial_friction (number_of_joints_);

    for (int i = 0; i < number_of_joints_; i++) initial_friction(i) = chain_.joint_static_friction[i];
    std::vector<double> resulting_torque_set(number_of_joints_);
    //TODO
    // select_non_moving_joints(m, initial_friction);
    // std::cout <<friction_torque_<< '\n';

    //Define resolution(step value) for each joint
    std::vector<double> resolution;
    for (int i = 0; i < number_of_joints_; i++){
        resolution.push_back(2 * initial_friction(i));
    }

    plot_file.open ("../src/Simulation/plot_data.txt");

    iterate_over_torques(m, initial_friction, resolution, 0, NUMBER_OF_STEPS, resulting_torque_set);

    for (int i = 0; i < number_of_joints_ + 1; i++) {
        if(i != number_of_joints_) plot_file << optimum_friction_tau_[i] <<" ";
        else plot_file << max_acc_energy;
    }
    final_acc_energy = max_acc_energy;
    plot_file.close();
}

void vereshchagin_with_friction::iterate_over_torques(
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

        KDL::SetToZero(tau_);
        KDL::Subtract(m.feedforward_torque, friction_torque_, tau_);

        qdd_ = m.qdd;

        int result = solver_.CartToJnt(
                     m.q, m.qd, qdd_, //qdd_ is overwritten by actual\resulting acceleration
                     m.end_effector_unit_constraint_forces,       // alpha
                     m.end_effector_acceleration_energy_setpoint, // beta
                     m.external_force,
                     tau_);  // tau_ is overwritten!
             assert(result == 0);

        solver_.get_link_acceleration(frame_acceleration_);
        solver_.get_transformed_link_acceleration(transformed_frame_acceleration_);
        solver_.get_link_inertias(articulated_body_inertia_);
        solver_.get_bias_force(bias_force_);
        solver_.get_control_torque(control_tau_);

        double acc_energy = compute_acc_energy(
                        frame_acceleration_,
                        articulated_body_inertia_,
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

        // Choose maximum value of Gauss function
        if (acc_energy > max_acc_energy){
            max_acc_energy = acc_energy;
            optimum_friction_tau_ = resulting_set;
            true_control_torques = control_tau_;
            true_joint_acc = qdd_;
            true_frame_acceleration_ = transformed_frame_acceleration_;
        }

        return;
    }

    else {
       for (int i = 0; i < steps; i++){
           resulting_set[joint_index] = -initial_friction(joint_index) +  (resolution[joint_index] * i);
           iterate_over_torques(m, initial_friction, resolution, joint_index + 1, steps, resulting_set);
       }
    }
}

//Calculate acceleration energy based on Gauss least constraint principle
double vereshchagin_with_friction::compute_acc_energy(
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

void vereshchagin_with_friction::select_non_moving_joints(motion_specification &m, KDL::JntArray &temp_friction_torques){

    for (int i = 0; i < number_of_joints_; i++) {
        //TODO check for the right 0 treshold!!!!! Best ask Sven for float inacuracy
        if (abs(m.qd(i)) > 0.01){
            temp_friction_torques(i) = 0;
        }
        else {
            std::cout <<"Not moving!!  "<< i << " " <<temp_friction_torques(i) << '\n';
        }
    }
}
