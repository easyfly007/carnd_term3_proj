#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <string>

using namespace std;

class GNB {
public:

	vector<string> possible_labels;


	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  	string predict(vector<double>);

 private:
 	vector<double> p_labels;
 	vector<vector<vector<double> > > p_X_over_Y;
 	// first level vec for each Y label type, which has size 3
 	// second level vec for each X features, which has size 4
 	// third level vec for mean and std for gaussion distribution for the specific Y lable and X feature

};

#endif



