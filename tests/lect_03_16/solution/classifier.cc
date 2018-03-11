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
	int data_cnt = data.size();
	int feat_cnt = data[0].size();

	int label_cnt = 3;
	vector<int> data_cnt_by_label(label_cnt, 0);
	for (int label_idx = 0; label_idx < label_cnt; label_idx ++)
	{
		vector<vector<double> >  dist_params_by_feature;
		for (int feat_idx = 0; feat_idx < feat_cnt; feat_idx ++)
		{
			vector<double> dist_params;
			dist_params.push_back(0.); // to save mean 
			dist_params.push_back(0.); // to save std
			dist_params_by_feature.push_back(dist_params);
		}
		p_X_over_Y.push_back(dist_params_by_feature);
	}

	for (int data_idx = 0; data_idx < data_cnt; data_idx ++)
	{
		string label = labels[data_idx];
		for (int label_idx = 0; label_idx < label_cnt; label_idx ++)
		{
			if (label == possible_labels[label_idx])
			{
				data_cnt_by_label[label_idx] ++;
				for (int feat_idx = 0; feat_idx < feat_cnt; feat_idx ++)
					p_X_over_Y[label_idx][feat_idx][0] += data[data_idx][feat_idx];
				break;
			}
		}
	}


	for (int label_idx = 0; label_idx < label_cnt; label_idx ++)
		p_labels[label_idx] = float(data_cnt_by_label[label_idx]/ float(data_cnt);

	// update means
	for (int feat_idx = 0; feat_idx < feat_cnt; feat_idx ++)
	{
		for (int label_idx = 0; label_idx < label_cnt; label_idx ++)
			p_X_over_Y[label_idx][feat_idx][0] /= float(data_cnt_by_label[label_idx]);
	}

	// add sum of std
	for (int data_idx = 0; data_idx < data_cnt; data_idx ++)
	{
		string label = labels[data_idx];
		for (int label_idx = 0; label_idx < label_cnt; label_idx ++)
		{
			if (possible_labels[label_idx] == label)
			{
				for (int feat_idx = 0; feat_idx < feat_cnt; feat_idx ++)
				{
					double mean = p_X_over_Y[label_idx][feat_idx][0];
					double std = (data_points[i] - mean) * (data_points[i] - mean);
				}
				double mean = std += (data_points[i] - mean) * (data_points[i] - mean);
				p_X_over_Y[label_idx][feat_idx][1] += mean;
				break;
			}
		}
	}

	for (int label_idx = 0; label_idx < label_cnt; label_idx ++)
	{
		for (int feat_idx = 0; feat_idx < feat_cnt; feat_idx ++)
			p_X_over_Y[label_idx][feat_idx][1] /= data_cnt_by_label[label_idx];
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