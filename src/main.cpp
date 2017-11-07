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

class Initialization {

public:
    KDL::Chain chaindyn;
    KDL::Twist* root_acc;
    KDL::JntArray* jointPoses;
    KDL::JntArray* jointRates;
    KDL::JntArray* jointAccelerations;
    KDL::JntArray* joint_ff_torques;
    KDL::JntArray* beta;
    KDL::Jacobian* alpha;
    KDL::Wrenches externalNetForce;
    double joint_inertia = 0;
    int numberOfConstraints = 0;

    Initialization(){

    }
    Initialization(KDL::Chain chaindyn, double joint_inertia){
        this -> chaindyn = chaindyn;
        this -> joint_inertia = joint_inertia;
    }

    void initialize_arm_configuration(){
        //These arrays of joint values contain actual and desired values
        //actual is generated by a solver and integrator
        //desired is given by an interpolator
        //error is the difference between desired-actual
        //in this test only the actual values are printed.
        jointPoses = new KDL::JntArray(chaindyn.getNrOfJoints());
        jointRates = new KDL::JntArray(chaindyn.getNrOfJoints());
        jointAccelerations = new KDL::JntArray(chaindyn.getNrOfJoints());
        joint_ff_torques = new KDL::JntArray(chaindyn.getNrOfJoints());

        // Initial arm position configuration/constraint
        jointPoses[0](0) = M_PI/2.0; // initial joint0 pose
        jointPoses[0](1) = M_PI/6.0;
        //j0=0.0, j1=pi/6.0 correspond to x = 0.2, y = -0.7464
        //j0=2*pi/3.0, j1=pi/4.0 correspond to x = 0.44992, y = 0.58636

        joint_ff_torques[0](0) = 0;
        joint_ff_torques[0](1) = 0;
    }

    //Definition of constraints and external disturbances
    void define_constraints(){

        numberOfConstraints = 1;

        KDL::Vector constrainXLinear(0.0, 0.0, 0.0);
        KDL::Vector constrainXAngular(0.0, 0.0, 0.0);

        KDL::Twist constraintForcesX(constrainXLinear, constrainXAngular);

        alpha = new KDL::Jacobian(numberOfConstraints);
        alpha -> setColumn(0, constraintForcesX);

        //Acceleration energy at  the end-effector
        beta = new KDL::JntArray(numberOfConstraints); //set to zero.
        //the task is to keep ee_acc to 0 in specified direction by constrain<>Angular/constrain<>Linear!
        beta[0](0) = 0.0;

        //arm root acceleration
        KDL::Vector linearAcc(0.0, -9.81, 0.0); //gravitational acceleration along Y
        KDL::Vector angularAcc(0.0, 0.0, 0.0);
        root_acc = new KDL::Twist(linearAcc, angularAcc);

        //external forces on the arm
        KDL::Vector externalForce1(0.0, 0.0, 0.0);
        KDL::Vector externalTorque1(0.0, 0.0, 0.0);
        KDL::Vector externalForce2(0.0, 0.0, 0.0);
        KDL::Vector externalTorque2(0.0, 0.0, 0.0);
        KDL::Wrench externalNetForce1(externalForce1, externalTorque1);
        KDL::Wrench externalNetForce2(externalForce2, externalTorque2);

        externalNetForce.push_back(externalNetForce1);
        externalNetForce.push_back(externalNetForce2);
    }

        //chain definition for vereshchagin's dynamic solver
    void create_kinematic_chain()
    {
        //last 3 inputs...input_scale, offset and joint inertia (d in paper)
        KDL::Joint rotJoint0 = KDL::Joint(KDL::Joint::RotZ, 1, 0, joint_inertia);
        KDL::Joint rotJoint1 = KDL::Joint(KDL::Joint::RotZ, 1, 0, joint_inertia);

        // RPY(roll,pitch,yaw) Rotation built from Roll-Pitch-Yaw angles
        KDL::Frame refFrame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
        KDL::Frame frame1(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.4, 0.0));
        KDL::Frame frame2(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.4, 0.0));

        //Frames desctibe pose of the segment tip, wrt joint frame
        KDL::Segment segment1 = KDL::Segment(rotJoint0, frame1);
        KDL::Segment segment2 = KDL::Segment(rotJoint1, frame2);

        //rotational inertia around symmetry axis of rotation
        KDL::RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        //spatial inertia
        // center of mass at the same position as tip  of segment???
        KDL::RigidBodyInertia inerSegment1(0.3, KDL::Vector(0.0, 0.4, 0.0), rotInerSeg1);
        KDL::RigidBodyInertia inerSegment2(0.3, KDL::Vector(0.0, 0.4, 0.0), rotInerSeg1);
        segment1.setInertia(inerSegment1);
        segment2.setInertia(inerSegment2);

        //adding segments in chain
        chaindyn.addSegment(segment1);
        chaindyn.addSegment(segment2);
    }
};

class Friction_enabled_vereshchagin
{
    KDL::Solver_Vereshchagin* solver;
    Initialization init_params;
    std::vector<KDL::Twist> sum_xDotdot;
    std::vector<KDL::ArticulatedBodyInertia> sum_H;
    KDL::Wrenches sum_U;

    //file for storing torque values and respective acc energy imposed by them
    std::ofstream plot_file;
    int numb_joints = 0, numb_segments = 0, numb_steps = 40;

public:
    KDL::JntArray* friction_torque;
    double max_acc_energy = 0;
    std::vector<double> optimum_torques;

    Friction_enabled_vereshchagin (Initialization parameters, std::vector<double> friction_tau, int numb_steps) {
        this -> numb_steps = numb_steps;
        init_params = parameters;
        numb_joints = init_params.chaindyn.getNrOfJoints();
        numb_segments = init_params.chaindyn.getNrOfSegments();

        friction_torque = new KDL::JntArray(friction_tau.size());
        friction_torque -> resize(numb_joints);
        sum_xDotdot.resize(numb_segments);
        sum_H.resize(numb_segments);
        sum_U.resize(numb_segments);
        optimum_torques.resize(numb_joints);

        for(int i = 0; i < numb_joints; i++){
            friction_torque[0](i) = friction_tau[i];
        }
        solver = new KDL::Solver_Vereshchagin(init_params.chaindyn, *init_params.root_acc, init_params.numberOfConstraints);
    }

    void cart_to_jnt() {

        plot_file.open ("/home/djole/Downloads/Master/R_&_D/KDL_GIT/Testing_repo/src/Simulation/plot_data.txt");

        //Define resolution(step value) for each joint
        int temp_numb_joints = numb_joints;
        std::vector<int> steps_set(numb_joints, numb_steps);
        std::vector<double> resulting_set(numb_joints);
        std::vector<double> resolution;

        for (int i = 0; i < numb_joints; i++){
            resolution.push_back((2 * friction_torque[0](i)) / numb_steps);
        }

        iterate_over_torques(resolution, 0, steps_set, resulting_set);

        for (int i = 0; i < numb_joints + 1; i++) {
            if(i != numb_joints){
                plot_file << optimum_torques[i] <<" ";
            }
            else{
                plot_file << max_acc_energy;
            }
        }
         plot_file.close();
         std::cout << "Max Gauss" << '\n';
         std::cout << max_acc_energy << '\n';
         std::cout << "Torques:" << '\n';
         std::cout << optimum_torques[0] << '\n';
         std::cout << optimum_torques[1] << '\n';
         std::cout << " " << '\n';
    }

private:

    void iterate_over_torques(const std::vector<double> resolution, int joint, const std::vector<int> steps_set, std::vector<double> resulting_set){

        if (joint >= numb_joints) {

            KDL::JntArray* new_ff_torques = new KDL::JntArray(numb_joints);
            KDL::JntArray* temp_friction_torques = new KDL::JntArray(numb_joints);

            for(int j = 0; j < numb_joints; j++){
                temp_friction_torques[0](j) = resulting_set[j];
            }

            //TODO
            //this is only relevant for Simulation?? BEcause there we get them as current
            // select_non_moving_joints(*temp_friction_torques);
            KDL::Add(*init_params.joint_ff_torques, *temp_friction_torques, *new_ff_torques);

            //TODO
            //Check if acc is changed/overwritten after each call of CartToJnt!!! Yes it is!!!!
            //Even it is not used further on in computs. inside library!!
            //Which means these acc are not accounted as input/desired values!!!!
            //It is only been rewritten by actual and those accual are used for compuation later on...in solver itself and for simulation
            //This overwriting is ok for simulation !!!
            //Maybe this is due to forward dyn part in Hybrid algorithm...if we dont specify constraints...
            //and only specify ff_torques...forward algo should give accual acc!!!!
            // maybe it is ok also for optimization procedure...because every new inputs does not influence calcs!!

            //But the question is: is  the overwriting of input torques by constraint torques ok for simulation???
            //Because the constraint forces are always included/defined separately in each iteration
            //constraint forces impose separate work than ff_force...as stated in Phd and Book
            //But the further question: is this due to inverse part of algor.....

            //After calling CartToJnt function, overwritten values are pased in calc_acc_energy
            //Which means Gauss is calculated with constarint torques!
            //Is original KDL implementation of test simulation  not correct due to this ?!
            //Maybe Azamat placed constraint torques here because of simulation....
            //to simulate how calculated constraint forces will influence motion...
            //of course if they exist...here in simullation we neeed to produce them artificialy!!

            //Further issues with only constraint torques as output are defined more in solver code!!!
            //Here is the torques values are reseted in each iteration~!

            int return_solver = solver -> CartToJnt(*init_params.jointPoses, *init_params.jointRates, *init_params.jointAccelerations, *init_params.alpha, *init_params.beta, init_params.externalNetForce, *new_ff_torques);

            if (return_solver == 0){
                solver -> get_link_acceleration(sum_xDotdot);
                solver -> get_link_inertias(sum_H);
                solver -> get_bias_force(sum_U);
             }

             else{
                 std::cout << "Error: unmatching matrix and array sizes" << '\n';
             }

             double acc_energy = compute_acc_energy(*new_ff_torques);

             //write data in the file
             for (int k = 0; k < numb_joints + 1; k++){
                 if(k != numb_joints){
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
            for (int i = 0; i < steps_set[joint]; i++){
                resulting_set[joint] = -friction_torque[0](joint) +  (resolution[joint]*(i+1));
                iterate_over_torques(resolution, joint + 1, steps_set, resulting_set);
            }
        }
    }

    //Calculate acceleration energy based on Gauss least constraint principle
    double compute_acc_energy(KDL::JntArray ff_torques) {
        //Talk with Sven rearding jointArray implementation, accesing and initialization!!!
        //Azamat wanted to work with row vectors?????
        // std::cout << friction_torque[0]<< '\n';

        double joint_sum = 0;
        double segment_sum = 0;
        double joint_inertia = init_params.joint_inertia;

        //check if here should be ff_torques+friction_torque of  both!
        //From the paper it seams that Q is ff_torque + friction_torque
        for (int j = 0; j < numb_joints; j++)  {
            joint_sum = joint_sum  + 0.5 * (joint_inertia * pow(init_params.jointAccelerations[0](j), 2)) - ff_torques(j) * init_params.jointAccelerations[0](j);
        }

        for (int i = 0; i < numb_segments; i++) {
            segment_sum += 0.5 * (dot(sum_xDotdot[i], sum_H[i] * sum_xDotdot[i]) + dot(sum_U[i], sum_xDotdot[i]));
        }

        return joint_sum + segment_sum;
    }

    void select_non_moving_joints(KDL::JntArray &temp_friction_torques){

        for (int i = 0; i < numb_joints; i++) {
            //TODO check for the right 0 treshold!!!!! Best ask Sven for float inacuracy
            //TODO test functioon by calling it with simulation
            //there is no sense calling it witout simulation
            //But it seams that works
            if (abs(init_params.jointRates[0](i)) > 0.01){
                temp_friction_torques(i) = 0;
            }
            else {
                std::cout <<"Not moving!!  "<< i << " " <<temp_friction_torques(i) << '\n';
            }
        }
    }
};

class Simulation
{
    KDL::Solver_Vereshchagin* original_solver;
    Initialization init_params;
    KDL::JntArray* jointPoses;
    KDL::JntArray* jointRates;
    KDL::JntArray* jointAccelerations;
    KDL::JntArray* friction_torque;
    double time_delta = 0.01;

public:

    Simulation(Initialization init_params, double time_delta) {
        this -> time_delta = time_delta;
        this -> init_params = init_params;
        original_solver = new KDL::Solver_Vereshchagin(init_params.chaindyn, *init_params.root_acc, init_params.numberOfConstraints);

        jointPoses = init_params.jointPoses;
        jointRates = init_params.jointRates;
        jointAccelerations = init_params.jointAccelerations;
    }

    //TODO make printing and integrate functions generic for N number of joints!!!!
    void print_current_state(double current_time)
    {
        printf("time              j0_pose       j1_pose        j0_rate       j1_rate         j0_acc        j1_acc \n");
        printf("%f          %f      %f       %f     %f       %f      %f\n", current_time, jointPoses[0](0), jointPoses[0](1), jointRates[0](0), jointRates[0](1), jointAccelerations[0](0), jointAccelerations[0](1));

    }

    KDL::JntArray step() {

        int return_solver = original_solver -> CartToJnt(jointPoses[0], jointRates[0], jointAccelerations[0], *init_params.alpha, *init_params.beta, init_params.externalNetForce, *init_params.joint_ff_torques);

        if (return_solver == 0){
            integrate();
            return *jointPoses;
           }

         else{
             std::cout << "Error: unmatching matrix and array sizes, initial poses returned" << '\n';
             return *jointPoses;
         }
    }

private:
    //TODO make printing and integrate functions generic for N number of joints!!!!
    void integrate() {
        // Integration(robot joint values for rates and poses; actual) at the given "instanteneous" interval for joint position and velocity.
        jointRates[0](0) = jointRates[0](0) + jointAccelerations[0](0) * time_delta; //Euler Forward
        jointPoses[0](0) = jointPoses[0](0) + (jointRates[0](0) - jointAccelerations[0](0) * time_delta / 2.0) * time_delta; //Trapezoidal rule
        jointRates[0](1) = jointRates[0](1) + jointAccelerations[0](1) * time_delta; //Euler Forward
        jointPoses[0](1) = jointPoses[0](1) + (jointRates[0](1) - jointAccelerations[0](1) * time_delta / 2.0) * time_delta;
    }

};

int main(int argc, char* argv[])
{
    //TODO Try to make it iteractive....user to select given options..not like this
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0] << " lower_limit_Torgue upper_limit_Torque number_of_samples simulation_on simulation_parameter" << std::endl;
        return 1;
    }

    const double joint_inertia = 0.1;
    KDL::Chain chaindyn_1;
    std::vector<double> friction_tau = boost::assign::list_of(atof(argv[1])) (atof(argv[2]));

    Initialization init_params_1(chaindyn_1, joint_inertia);
    init_params_1.create_kinematic_chain();
    init_params_1.initialize_arm_configuration();
    init_params_1.define_constraints();

    Friction_enabled_vereshchagin friction_solver(init_params_1, friction_tau, atof(argv[3]));
    friction_solver.cart_to_jnt();

    //Boolean for choice of simulation
    bool simulation_on;
    std::stringstream ss(argv[4]);
    if(!(ss >> std::boolalpha >> simulation_on)) {
        std::cout << "Last argument must be boolean\n" << '\n';
    }

    if(simulation_on){
        //Time required to complete the task
        double simulation_time = atof(argv[5]);
        double time_delta = 0.01;
        //Here if we use the same instance of Initialization object
        //Initial values first need to  be reseted
        //Because they are rewritten by original solver in previous usage (optimization procedure in extended)
        //Maybe there is another method...beter one than the followning
        KDL::Chain chaindyn_2;
        Initialization init_params_2(chaindyn_2, joint_inertia);
        init_params_2.create_kinematic_chain();
        init_params_2.define_constraints();
        init_params_2.initialize_arm_configuration();

        Simulation simulate(init_params_2, time_delta);
        KDL::JntArray joint_poses;

        //File for storing joint values from arm simulation
        std::ofstream my_file;
        my_file.open ("/home/djole/Downloads/Master/R_&_D/KDL_GIT/Testing_repo/src/Simulation/joint_poses.txt");

        //TODO make printing and integrate functions generic for N number of joints!!!!
        for (double i = 0.0; i < simulation_time; i += time_delta) {
            joint_poses = simulate.step();
            my_file << joint_poses(0) <<" "<< joint_poses(1) <<"\n";
            // simulate.print_current_state(i);
        }
        my_file.close();
    }

	return 0;
}
