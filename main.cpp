#include <kdl/chainidsolver_vereshchagin.hpp>

int main(int argc, char **argv)
{
	KDL::Chain chain;
	KDL::ChainIdSolver_Vereshchagin solver(chain, KDL::Twist(KDL::Vector(0.0, 0.0, -9.81), KDL::Vector(0.0, 0.0, 0.0)), 6);

	return 0;
}
