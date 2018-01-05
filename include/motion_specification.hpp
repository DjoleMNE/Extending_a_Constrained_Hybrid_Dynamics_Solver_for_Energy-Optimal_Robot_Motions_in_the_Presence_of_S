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

#ifndef MOTION_SPECIFICATION_HPP
#define MOTION_SPECIFICATION_HPP
#include <solver_vereshchagin.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <Eigen/StdVector>

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

			  };

		~motion_specification()
	    {
	    };

		KDL::JntArray q;
		KDL::JntArray qd;
		KDL::JntArray qdd; // as input to original solver, overwritten during call!
		KDL::JntArray feedforward_torque;
		KDL::Jacobian end_effector_unit_constraint_forces;
		KDL::JntArray end_effector_acceleration_energy_setpoint;
		KDL::Wrenches external_force;
};
#endif
