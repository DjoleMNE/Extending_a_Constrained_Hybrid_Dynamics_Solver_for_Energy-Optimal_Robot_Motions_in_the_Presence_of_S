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
using namespace std;

using namespace KDL;
KDL::Chain chaindyn;

//File for storing joint values from arm simulation
ofstream myfile;

//file for storing torque values and respective acc energy imposed by them
ofstream plot_file;

//Boolean for choice of simulation
bool simulation_on;

//Vectors of algorithm outputs
vector<Twist> sum_xDotdot;

//Inertia is Eigen matrix type!!!
vector<ArticulatedBodyInertia> sum_H;
Wrenches sum_U;

//Time required to complete the task move(frameinitialPose, framefinalPose)
double _simulation_param = 10;
double taskTimeConstant = 0.1;
double simulationTime=1;
double timeDelta = 0.01;

//joint inertias
double d[] = {0.1, 0.1};

//These arrays of joint values contain actual and desired values
//actual is generated by a solver and integrator
//desired is given by an interpolator
//error is the difference between desired-actual
//in this test only the actual values are printed.
const int k = 1;
JntArray jointPoses[k];
JntArray jointRates[k];
JntArray jointAccelerations[k];
JntArray jointTorques[k];
Wrenches externalNetForce;

//Creating model of the arm
void setUp()
{
    srand((unsigned)time( NULL ));

    //chain definition for vereshchagin's dynamic solver
    //......................................

    //last 3 inputs...input_scale, offset and joint inertia(d in paper)
    Joint rotJoint0 = Joint(Joint::RotZ, 1, 0, d[0]);
    Joint rotJoint1 = Joint(Joint::RotZ, 1, 0, d[1]);

    // RPY(roll,pitch,yaw) Rotation built from Roll-Pitch-Yaw angles
    Frame refFrame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));

    //Frames desctibe pose of the segment tip, wrt joint frame
    Segment segment1 = Segment(rotJoint0, frame1);
    Segment segment2 = Segment(rotJoint1, frame2);

    //rotational inertia around symmetry axis of rotation
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    //spatial inertia
    // center of mass at the same position as tip  of segment???
    RigidBodyInertia inerSegment1(0.3, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(0.3, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);

    //chain
    chaindyn.addSegment(segment1);
    chaindyn.addSegment(segment2);

	// // Motoman SIA10 Chain (for IK singular value tests)
	// motomansia10.addSegment(Segment(Joint(Joint::None),
	// 								Frame::DH_Craig1989(0.0, 0.0, 0.36, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame::DH_Craig1989(0.0, M_PI_2, 0.0, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame::DH_Craig1989(0.0, -M_PI_2, 0.36, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame::DH_Craig1989(0.0, M_PI_2, 0.0, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame::DH_Craig1989(0.0, -M_PI_2, 0.36, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame::DH_Craig1989(0.0, M_PI_2, 0.0, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame::DH_Craig1989(0.0, -M_PI_2, 0.0, 0.0)));
	// motomansia10.addSegment(Segment(Joint(Joint::RotZ),
	// 								Frame(Rotation::Identity(),Vector(0.0,0.0,0.155))));
}

//print layouts for description of motion quantities
void printlayouts(){
    // const std::string msg = "Assertion failed, check matrix and array sizes";
    std::cout << "Input to algorithm" << '\n';
    std::cout << " " << '\n';
    printf("%s          %s      %s         %s     %s       %s         %s      %s      %s\n", "Time for Task  Constant", "j0_pose", "j1_pose", "j0_rate", "j1_rate", "j0_acc", "j1_acc", "j0_torque", "j1_torque");
    printf("%f                        %f      %f       %f     %f       %f      %f     %f      %f\n", taskTimeConstant, jointPoses[0](0), jointPoses[0](1), jointRates[0](0), jointRates[0](1), jointAccelerations[0](0), jointAccelerations[0](1), jointTorques[0](0), jointTorques[0](1));
    std::cout << " " << '\n';
    std::cout << "Output of algorithm: " << '\n';
    std::cout << " " << '\n';
}

//Simulate motion of the arm to get joint positon, velosity and accelerations values
void simulate_motion(Solver_Vereshchagin solver, Jacobian alpha, JntArray betha){
    myfile.open ("/home/djole/Downloads/Master/R_&_D/KDL_GIT/Testing_repo/src/Simulation/joint_poses.txt");
    simulationTime = _simulation_param*taskTimeConstant;

    printf("time              j0_pose       j1_pose        j0_rate       j1_rate         j0_acc        j1_acc       j0_constraintTau         j1_constraintTau \n");

    for (double t = 0.0; t <= simulationTime; t = t + timeDelta)
    {
        int return_solver = solver.CartToJnt(jointPoses[0], jointRates[0], jointAccelerations[0], alpha, betha, externalNetForce, jointTorques[0]);

        if (return_solver == 0){
            //Integration(robot joint values for rates and poses; actual) at the given "instanteneous" interval for joint position and velocity.
            jointRates[0](0) = jointRates[0](0) + jointAccelerations[0](0) * timeDelta; //Euler Forward
            jointPoses[0](0) = jointPoses[0](0) + (jointRates[0](0) - jointAccelerations[0](0) * timeDelta / 2.0) * timeDelta; //Trapezoidal rule
            jointRates[0](1) = jointRates[0](1) + jointAccelerations[0](1) * timeDelta; //Euler Forward
            jointPoses[0](1) = jointPoses[0](1) + (jointRates[0](1) - jointAccelerations[0](1) * timeDelta / 2.0) * timeDelta;
            myfile <<jointPoses[0](0)<<" "<<jointPoses[0](1)<<"\n";
            printf("%f          %f      %f       %f     %f       %f      %f     %f                 %f\n", t, jointPoses[0](0), jointPoses[0](1), jointRates[0](0), jointRates[0](1), jointAccelerations[0](0), jointAccelerations[0](1), jointTorques[0](0), jointTorques[0](1));
           }

         else{
             std::cout << "Error: unmatching matrix and array sizes" << '\n';
         }
    }
    myfile.close();
}

//Calculate acceleration energy based on Gauss least constraint principle
//Lower G
// vector istead of array...
double calculate_Gauss(double ff_torques[]){
    // number_to
    double number_of_joints = chaindyn.getNrOfJoints();
    double number_of_segments = chaindyn.getNrOfSegments();

    double joint_sum = 0;
    double segment_sum=0;

    for (int j = 0; j < number_of_joints; j++)  {
        joint_sum = joint_sum  + 0.5 * (d[j] * pow(jointAccelerations[0](j), 2)) - ff_torques[j] * jointAccelerations[0](j);
    }

    for (int i = 0; i < number_of_segments; i++) {
        segment_sum += 0.5 * (dot(sum_xDotdot[i], sum_H[i] * sum_xDotdot[i]) + dot(sum_U[i], sum_xDotdot[i]));

    }
    //space in code!!!!!!!!!!!
    return joint_sum + segment_sum;
}

// Evaluate motion quantities required for calculation of final acceleration energy
void evaluate_motion(Solver_Vereshchagin solver, Jacobian alpha, JntArray betha, double ff_torques[], double increment){
    printf("               s1_acc                                  s1_acc                                   j0_acc            j1_acc           j0_Tau_ff         j1_Tau_ff \n");
    plot_file.open ("/home/djole/Downloads/Master/R_&_D/KDL_GIT/Testing_repo/src/Simulation/plot_data.txt");

    double max_Gauss = 0;

    //use vector....this does not givee size dude!!!!!!!
    double input_torques[2];

    //make it for N number of joints!!!
    // double resolution = 2 * f_torques[0] / numb_steps;
    // for (int i = 0; i < ...; i++) {
    //     double current_torque = -f_torques[0] + i * resolution;
    // }

    for (double i = ff_torques[0]; i <= ff_torques[1]; i=i+increment)
    {
        for (double j = ff_torques[0]; j <= ff_torques[1]; j=j+increment)
        {
            jointTorques[0](0) = i;
            jointTorques[0](1) = j;

            int return_solver = solver.CartToJnt(jointPoses[0], jointRates[0], jointAccelerations[0], alpha, betha, externalNetForce, jointTorques[0]);

            if (return_solver == 0){
                solver.getLinkAcceleration(sum_xDotdot);
                solver.getLinkAcceleration(sum_H);
                solver.getBiasForce(sum_U);
             }

             else{
                 std::cout << "Error: unmatching matrix and array sizes" << '\n';
             }

             double acc_energy = calculate_Gauss(ff_torques);

             //write data in the file
             plot_file <<i<<" "<<j<<" "<<acc_energy<<"\n";

             if (acc_energy > max_Gauss){
                 max_Gauss = acc_energy;
                 input_torques[0] = i;
                 input_torques[1] = j;
             }

        }
    }
    // plot_file <<ff_torques[0]<<" "<<ff_torques[0]<<" "<<max_Gauss<<"\n";
    plot_file.close();
    std::cout << "Max Gauss" << '\n';
    std::cout << max_Gauss << '\n';
    std::cout << "Torques:" << '\n';
    std::cout << input_torques[0] << '\n';
    std::cout << input_torques[1] << '\n';
    std::cout << " " << '\n';
}

void VereshchaginTest(double ff_torques[], double torgue_increment, bool simulation_on, double simulation_param)
{
    //~Definition of constraints and external disturbances
    //-------------------------------------------------------------------------------------//
    Vector constrainXLinear(0.0, 0.0, 0.0);
    Vector constrainXAngular(0.0, 0.0, 0.0);
    // Vector constrainYLinear(0.0, 0.0, 0.0);
    // Vector constrainYAngular(0.0, 0.0, 0.0);
    //Vector constrainZLinear(0.0, 0.0, 0.0);
    //Vector constrainZAngular(0.0, 0.0, 0.0);

    Twist constraintForcesX(constrainXLinear, constrainXAngular);
    // Twist constraintForcesY(constrainYLinear, constrainYAngular);
    //Twist constraintForcesZ(constrainZLinear, constrainZAngular);

    Jacobian alpha(1);
    alpha.setColumn(0, constraintForcesX);
    //alpha.setColumn(0, constraintForcesZ);

    //Acceleration energy at  the end-effector
    JntArray betha(1); //set to zero.

    //the task is to keep ee_acc to 0 in specified direction by constrain<>Angular/constrain<>Linear!
    betha(0) = 0.0;
    //betha(1) = 0.0;
    //betha(2) = 0.0;

    //arm root acceleration
    //Why not -9.81???????? is y-axis reversed  ????
    Vector linearAcc(0.0, 9.81, 0.0); //gravitational acceleration along Y
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist twist1(linearAcc, angularAcc);

    //external forces on the arm
    Vector externalForce1(0.0, 0.0, 0.0);
    Vector externalTorque1(0.0, 0.0, 0.0);
    Vector externalForce2(0.0, 0.0, 0.0);
    Vector externalTorque2(0.0, 0.0, 0.0);
    Wrench externalNetForce1(externalForce1, externalTorque1);
    Wrench externalNetForce2(externalForce2, externalTorque2);
    externalNetForce.push_back(externalNetForce1);
    externalNetForce.push_back(externalNetForce2);

    //Definition of solver and initial configuration
    //-------------------------------------------------------------------------------------//
    int numberOfConstraints = 1;

    Solver_Vereshchagin constraintSolver(chaindyn, twist1, numberOfConstraints);

    for (int i = 0; i < k; i++)
    {
        JntArray jointValues(chaindyn.getNrOfJoints());
        jointPoses[i] = jointValues;
        jointRates[i] = jointValues;
        jointAccelerations[i] = jointValues;
        jointTorques[i] = jointValues;
    }

    // Initial arm position configuration/constraint
    JntArray jointInitialPose(chaindyn.getNrOfJoints());
    // jointInitialPose(0) = 0.0; // initial joint0 pose
    jointInitialPose(0) = M_PI/2.0; // initial joint0 pose
    jointInitialPose(1) = M_PI/6.0;
    //j0=0.0, j1=pi/6.0 correspond to x = 0.2, y = -0.7464
    //j0=2*pi/3.0, j1=pi/4.0 correspond to x = 0.44992, y = 0.58636

    //actual
    jointPoses[0](0) = jointInitialPose(0);
    jointPoses[0](1) = jointInitialPose(1);
    jointTorques[0](0) = ff_torques[0];
    jointTorques[0](1) = ff_torques[1];

    //Definition of process main loops
    //-------------------------------------------------------------------------------------//
    printlayouts();

    if (simulation_on){
        _simulation_param = simulation_param;
        simulate_motion(constraintSolver, alpha, betha);
    }

    else{
        evaluate_motion(constraintSolver, alpha, betha, ff_torques, torgue_increment);
    }

}

int main(int argc, char* argv[])
{
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0] << " lower_limit_Torgue upper_limit_Torque Torgue_increment simulation_on simulation_parameter" << std::endl;
        return 1;
    }

    std::stringstream ss(argv[4]);

    if(!(ss >> std::boolalpha >> simulation_on)) {
        std::cout << "Last argument must be boolean\n" << '\n';
    }

    double ff_torques[] = {atof (argv[1]), atof (argv[2])};

	setUp();

	VereshchaginTest(ff_torques, atof (argv[3]), simulation_on, atof (argv[5]));

	return 0;
}