#include "App.h"

const std::string gWinTitle = "CPSC 426 Assignment 1";
int gWinWidth = 800;
int gWinHeight = 450;

void EigenExamples()
{
	// simple examples for working with Eigen
	std::cout << "Eigen Examples:" << std::endl;

	tVector x0 = tVector(1, 2, 3, 4); // tVector is a 4D vector
	tMatrix M0 = tMatrix::Identity(); // tMatrix is a 4x4 matrix
	tVector y0 = M0 * x0;

	std::cout << "x0" << std::endl;
	std::cout << x0 << std::endl << std::endl;
	std::cout << "M0" << std::endl;
	std::cout << M0 << std::endl << std::endl;
	std::cout << "y0" << std::endl;
	std::cout << y0 << std::endl << std::endl;
	std::cout << std::endl;

	// individual entries and vectors can be accessed using their indices
	std::cout << "x0(0) = " << x0(0) << std::endl;
	std::cout << "x0(1) = " << x0(1) << std::endl;
	std::cout << "x0(2) = " << x0(2) << std::endl;
	std::cout << "x0(3) = " << x0(3) << std::endl;
	std::cout << std::endl;
	std::cout << "M0(0, 0) = " << M0(0, 0) << std::endl;
	std::cout << "M0(0, 1) = " << M0(0, 1) << std::endl;
	std::cout << "M0(1, 0) = " << M0(1, 0) << std::endl;
	std::cout << "M0(1, 1) = " << M0(1, 1) << std::endl;
	std::cout << std::endl;

	// can also specify arbitrary length vectors and matrices
	Eigen::VectorXd x1(6); // 8D vector
	Eigen::MatrixXd M1(6, 6); // 8x8 matrix

	// they can be initialized using <<
	x1 << 6, 5, 4, 3, 2, 1;
	M1 << 1, 2, 3, 4, 5, 6,
		  7, 8, 9, 10, 11, 12,
		  13, 14, 15, 16, 17, 18,
		  19, 20, 21, 22, 23, 24,
		  25, 26, 27, 28, 29, 30,
		  31, 32, 33, 34, 35, 36;
	Eigen::VectorXd y1 = M1 * x1;

	std::cout << "x1" << std::endl;
	std::cout << x1 << std::endl << std::endl;
	std::cout << "M1" << std::endl;
	std::cout << M1 << std::endl << std::endl;
	std::cout << "y1" << std::endl;
	std::cout << y1 << std::endl << std::endl;

	// you can also multiply matrices together
	Eigen::MatrixXd M2 = Eigen::MatrixXd::Random(5, 5);
	Eigen::MatrixXd M3 = Eigen::MatrixXd::Identity(5, 5);
	Eigen::MatrixXd M4 = M2 * M3;

	std::cout << "M2" << std::endl;
	std::cout << M2 << std::endl << std::endl;
	std::cout << "M3" << std::endl;
	std::cout << M3 << std::endl << std::endl;
	std::cout << "M4" << std::endl;
	std::cout << M4 << std::endl << std::endl;
}

int main(int argc, char ** argv) {

	EigenExamples();

	try {
		nanogui::init();
		{
			nanogui::ref<cApp> app = new cApp(gWinWidth, gWinHeight, gWinTitle);
			app->Init();

			app->drawAll();
			app->setVisible(true);

			int mill_per_frame = static_cast<int>(1000 / app->GetFPS());
			nanogui::mainloop(mill_per_frame);
		}
		nanogui::shutdown();
	}
	catch (const std::runtime_error &e)
	{
		std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
		std::cerr << error_msg << std::endl;
		return -1;
	}

	return 0;
}
