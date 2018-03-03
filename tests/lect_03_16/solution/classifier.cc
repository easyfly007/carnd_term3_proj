#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

#include <math.h> // for sqrt
/**
 * Initializes GNB
 */
// vector<double> data_points = get_value_by_ylabel_by_xfeature(vector<vector<double> > & data , vector<string> labels &,
// 	int label_idx, int feat_idx, vector<string> possible_labels &);

vector<double> get_value_by_ylabel_by_xfeature(vector<vector<double> > & data, vector<string> & labels,
	int label_idx, int feat_idx, vector<string> & possible_labels){
	vector<double> data_points;

	return data_points;
}

GNB::GNB() {
	possible_labels.push_back(string("left"));
	possible_labels.push_back(string("keep"));
	possible_labels.push_back(string("right"));
	p_labels.push_back(0.0);
	p_labels.push_back(0.0);
	p_labels.push_back(0.0);

}

GNB::~GNB() {

}

void GNB::train(vector<vector<double> > data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/
	int data_size = data.size();
	int feat_size = data[0].size();

	vector<int> cnt_by_label(3, 0);

	for (int i = 0; i < data_size; i ++)
	{
		string label = labels[i];
		for (int j = 0; j < possible_labels.size(); j ++)
		{
			if (label == possible_labels[j])
			{
				cnt_by_label[j] ++;
				break;
			}
		}
	}

	for (int i = 0; i < p_labels.size(); i ++)
	{
		p_labels[i] = float(cnt_by_label[i]) / float(data_size);
	}


	// calc p(X| Y)
	for (int label_idx = 0 ; label_idx < possible_labels.size(); label_idx ++)
	{
		vector<vector<double> > dist_params_by_feature;

		for (int feat_idx = 0; feat_idx < feat_size; feat_idx ++)
		{
			vector<double> dist_params;
			double mean, std;
			vector<double> data_points = get_value_by_ylabel_by_xfeature(data, labels, label_idx, feat_idx, possible_labels);
			double sum = 0.0;
			for (int i = 0; i < data_points.size(); i ++)
			{
				sum += data_points[i];
			}
			mean = sum / data_points.size();
			
			std = 0.0;
			for (int i = 0; i < data_points.size(); i ++)
			{
				std += (data_points[i] - mean) * (data_points[i] - mean);
			}
			std /= data_points.size();
			dist_params.push_back(mean);
			dist_params.push_back(std);
			dist_params_by_feature.push_back(dist_params);
		}
		p_X_over_Y.push_back(dist_params_by_feature);
	}


}

string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/
	int  max_label = 0;
	double max_prob = 0.0;
	for (int i = 0; i < possible_labels.size(); i ++)
	{
		double this_prob = p_labels[i];
		for (int j = 0; j < sample.size(); j ++)
		{
			double x = sample[j];
			double mean = p_X_over_Y[i][j][0];
			double std  = p_X_over_Y[i][j][1];
			this_prob *= 1.0/ sqrt(std) * exp(- (x - mean) * (x-mean) / (2. * std) );
		}
		if (this_prob > max_prob)
		{
			max_prob = this_prob;
			max_label = i;
		}
	}


	return this->possible_labels[max_label];

}