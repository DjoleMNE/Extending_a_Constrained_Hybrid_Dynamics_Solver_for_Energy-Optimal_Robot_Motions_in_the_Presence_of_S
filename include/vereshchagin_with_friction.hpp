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

#ifndef VERESHCHAGIN_WITH_FRICTION_HPP
#define VERESHCHAGIN_WITH_FRICTION_HPP
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

class vereshchagin_with_friction {

    public:

		KDL::JntArray true_control_torques;
		KDL::JntArray true_joint_acc;
        double final_acc_energy;

        vereshchagin_with_friction(
                const extended_kinematic_chain &chain,
                const KDL::Twist &acc_root,
                int number_of_constraints);

        ~vereshchagin_with_friction()
        {
        };

		void solve(motion_specification &m);

	private:
		void iterate_over_torques(
			motion_specification &m,
			const KDL::JntArray initial_friction,
			const std::vector<double> resolution,
			int joint_index,
			const int steps,
			std::vector<double> resulting_set);

		//Calculate acceleration energy based on Gauss least constraint principle
		double compute_acc_energy(
				const std::vector<KDL::Twist> &xdd,
				const std::vector<KDL::ArticulatedBodyInertia> &h,
				const KDL::Wrenches &bias_force,
				const KDL::JntArray &qdd,
				const KDL::JntArray &tau);

		void select_non_moving_joints(motion_specification &m, KDL::JntArray &temp_friction_torques);

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
		KDL::JntArray control_tau_;

		double max_acc_energy;
		std::vector<double> optimum_friction_tau_;

		std::ofstream plot_file;

};
#endif
