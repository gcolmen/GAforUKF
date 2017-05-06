
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
Function to return a random number in the [0, 1] interval
*/
double rand01()
{
	return ((double)rand() / (RAND_MAX));
}

/*
Class for the individual.
*/
class solution {
public:
	//The fitness value for this individual
	double fitness = 0;
	//The individual data (containing the parameters that we're trying to optimize)
	UKF ukf;
	void solution::randInit();
};

/*
Function to randomly set the parameters (of this object) by adding noise to the parameters of interest.

*/
void solution::randInit()
{
	ukf.std_a_ += (rand01() * 0.40 - 0.20) * ukf.std_a_;
	ukf.std_yawdd_ += (rand01() * 0.40 - 0.20) * ukf.std_yawdd_;
	/*ukf.std_laspx_ += (rand01() * 0.40 - 0.20) * ukf.std_laspx_;
	ukf.std_laspy_ += (rand01() * 0.40 - 0.20) * ukf.std_laspy_;
	ukf.std_radr_ += (rand01() * 0.40 - 0.20) * ukf.std_radr_;
	ukf.std_radphi_ += (rand01() * 0.40 - 0.20) * ukf.std_radphi_;
	ukf.std_radrd_ += (rand01() * 0.40 - 0.20) * ukf.std_radrd_;
	ukf.std_radr_ += (rand01() * 0.40 - 0.20) * ukf.std_radr_;*/

	for (int i = 0; i < ukf.n_x_; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			ukf.P_(i, j) += (rand01() * 0.40 - 0.20) * ukf.P_(i, j);
		}
		
	}
	for (int i = 0; i < ukf.n_x_; i++)
	{
		for (int j = i + 1; j < ukf.n_x_; j++)
		{
			ukf.P_(i, j) = ukf.P_(j, i);
		}
	}
	
}


/*
Struct used to order the individuals in the vector using the fitness value.*/
struct ascendingSol
{
	bool operator()(solution const &a, solution const &b) const { return a.fitness < b.fitness; }
};


/*
Function to randomly mutate an individual. 
With probability of 70% each gen is mutated. The mutation adds some noise to the current gen value.
Returns the mutated individual.
*/
solution mutate(solution offspring)
{
	if (rand01() < 0.7)
		offspring.ukf.std_a_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_a_;
	if (rand01() < 0.7)
		offspring.ukf.std_yawdd_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_yawdd_;
	/*if (rand01() < 0.5)
		offspring.ukf.std_laspx_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_laspx_;
	if (rand01() < 0.5)
		offspring.ukf.std_laspy_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_laspy_;
	if (rand01() < 0.5)
		offspring.ukf.std_radr_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_radr_;
	if (rand01() < 0.5)
		offspring.ukf.std_radphi_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_radphi_;
	if (rand01() < 0.5)
		offspring.ukf.std_radrd_ += (rand01() * 0.40 - 0.20) * offspring.ukf.std_radrd_;*/
	if (rand01() < 0.7)
	{
		for (int i = 0; i < offspring.ukf.n_x_; i++)
		{
			for (int j = 0; j <= i; j++)
			{
				offspring.ukf.P_(i, j) += (rand01() * 0.40 - 0.20) * offspring.ukf.P_(i, j);
			}

		}
		for (int i = 0; i < offspring.ukf.n_x_; i++)
		{
			for (int j = i + 1; j < offspring.ukf.n_x_; j++)
			{
				offspring.ukf.P_(i, j) = offspring.ukf.P_(j, i);
			}
		}
	}
	return offspring;
}

/*
Function to combine (or cross) two individuals.
With a probability of 50% it copies a gen from the father or from the mother
to the offspring.
Returns the offspring individual.
*/
solution cross(solution sol, solution sol2)
{
	solution offspring;
	if (rand01() < 0.5)
		offspring.ukf.std_a_ = sol.ukf.std_a_;
	else
		offspring.ukf.std_a_ = sol2.ukf.std_a_;
	if (rand01() < 0.5)
		offspring.ukf.std_yawdd_ = sol.ukf.std_yawdd_;
	else
		offspring.ukf.std_yawdd_ = sol2.ukf.std_yawdd_;
	/*if (rand01() < 0.5)
		offspring.ukf.std_laspx_ = sol.ukf.std_laspx_;
	else
		offspring.ukf.std_laspx_ = sol2.ukf.std_laspx_;
	if (rand01() < 0.5)
		offspring.ukf.std_laspy_ = sol.ukf.std_laspy_;
	else
		offspring.ukf.std_laspy_ = sol2.ukf.std_laspy_;
	if (rand01() < 0.5)
		offspring.ukf.std_radr_ = sol.ukf.std_radr_;
	else
		offspring.ukf.std_radr_ = sol2.ukf.std_radr_;
	if (rand01() < 0.5)
		offspring.ukf.std_radphi_ = sol.ukf.std_radphi_;
	else
		offspring.ukf.std_radphi_ = sol2.ukf.std_radphi_;
	if (rand01() < 0.5)
		offspring.ukf.std_radrd_ = sol.ukf.std_radrd_;
	else
		offspring.ukf.std_radrd_ = sol2.ukf.std_radrd_;*/
	if (rand01() < 0.5)
		offspring.ukf.P_ = sol.ukf.P_;
	else
		offspring.ukf.P_ = sol2.ukf.P_;

	return offspring;
}

/*

Function compute the fitness of an individual.
It executes the measurement process (predict and update)
And returns the RMSE value (fitness)
*/
double computeFitness(UKF ukf, vector<MeasurementPackage> measurement_pack_list, vector<GroundTruthPackage> gt_pack_list)
{
	// used to compute the RMSE later
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	size_t number_of_measurements = measurement_pack_list.size();

	for (size_t k = 0; k < number_of_measurements; ++k) {
		// Call the UKF-based fusion
		ukf.ProcessMeasurement(measurement_pack_list[k]);

		// convert ukf x vector to cartesian to compare to ground truth
		VectorXd ukf_x_cartesian_ = VectorXd(4);

		float x_estimate_ = ukf.x_(0);
		float y_estimate_ = ukf.x_(1);
		float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
		float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

		ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

		estimations.push_back(ukf_x_cartesian_);
		ground_truth.push_back(gt_pack_list[k].gt_values_);

	}

	// compute the accuracy (RMSE)
	Tools tools;
	VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
	cout << "RMSE" << endl << rmse << endl;
	return rmse.sum();
}

int main(int argc, char* argv[]) {

	//check_arguments(argc, argv);

	string in_file_name_ = argv[1];
	ifstream in_file_(in_file_name_.c_str(), ifstream::in);

	string out_file_name_ = argv[2];
	//ofstream out_file_(out_file_name_.c_str(), ofstream::out);

	//check_files(in_file_, in_file_name_, out_file_, out_file_name_);

	/**********************************************
	*  Set Measurements                          *
	**********************************************/

	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;

	string line;

	// prep the measurement packages (each line represents a measurement at a
	// timestamp)
	while (getline(in_file_, line)) {
		string sensor_type;
		MeasurementPackage meas_package;
		GroundTruthPackage gt_package;
		istringstream iss(line);
		long long timestamp;

		// reads first element from the current line
		iss >> sensor_type;

		if (sensor_type.compare("L") == 0) {
			// laser measurement

			// read measurements at this timestamp
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float px;
			float py;
			iss >> px;
			iss >> py;
			meas_package.raw_measurements_ << px, py;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}
		else if (sensor_type.compare("R") == 0) {
			// radar measurement

			// read measurements at this timestamp
			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			float ro;
			float phi;
			float ro_dot;
			iss >> ro;
			iss >> phi;
			iss >> ro_dot;
			meas_package.raw_measurements_ << ro, phi, ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}

		// read ground truth data to compare later
		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		gt_package.gt_values_ = VectorXd(4);
		gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
		gt_pack_list.push_back(gt_package);
	}

	//Start GA

	vector<solution> inds;
	int N = 80;
	//init populaiton
	for (int i = 0; i < N; i++)
	{
		solution sol;
		sol.randInit();
		sol.fitness = computeFitness(sol.ukf, measurement_pack_list, gt_pack_list);
		inds.push_back(sol);
		
	}

	//sort descending
	std::sort(inds.begin(), inds.end(), ascendingSol());
	int GENERATIONS = 1000;
	int gen = 0;
	double minFitness = LONG_MAX;
	solution best;
	do {

		for (int i = 0; i < 8; i++)
		{
			// Create a UKF instance
			//UKF ukf;
			int ind = rand01() * 10;//from the best 10 choose
			solution sol = inds.at(ind);
			ind = rand01() * 10;//from the best 10 choose
			solution sol2 = inds.at(ind);
			//crossing
			solution offspring = cross(sol, sol2);
			//fitness
			offspring.fitness = computeFitness(offspring.ukf, measurement_pack_list, gt_pack_list);

			ind = 50 - rand01() * 10;//from the worst 10 choose
			inds.at(ind) = offspring;
			if (rand01() < 0.5)
			{
				offspring = mutate(offspring);
				//fitness
				offspring.fitness = computeFitness(offspring.ukf, measurement_pack_list, gt_pack_list);
				int mutInd;
				do {
					mutInd = 50 - rand01() * 10;//from the worst 10 choose
				} while (mutInd == ind);//To avoid replacing the other offspring
				inds.at(ind) = offspring;
			}
		}//end for new individuals
		 //sort descending
		std::sort(inds.begin(), inds.end(), ascendingSol());
		if (inds.at(0).fitness < minFitness)
		{
			minFitness = inds.at(0).fitness;
			best = inds.at(0);
			cout << "New min fitness: " << minFitness << endl;
			cout << "Std a" << best.ukf.std_a_ << endl;
			cout << "Std YAWDD" << best.ukf.std_yawdd_ << endl;
			cout << "P " << best.ukf.P_ << endl;
		}
		gen++;
		cout << "Generation " << gen << endl;
	} while (gen < GENERATIONS);
	cout << "Min fitness: " << minFitness << endl;
	cout << "Std a" << best.ukf.std_a_ << endl;
	cout << "Std YAWDD" << best.ukf.std_yawdd_ << endl;
	cout << "P " << best.ukf.P_ << endl;
	cout << "Done!" << endl;
	cin >> gen;
	return 0;
}