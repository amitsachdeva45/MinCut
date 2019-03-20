#ifndef SEG_SEGMENT_H
#define SEG_SEGMENT_H

#include <opencv2/opencv.hpp>
#include "extra.h"

double calculateMean(Area ar, std::vector<std::vector<double> > enr);

double calculateVariance(Area ar, std::vector<std::vector<double> > energy, double mean);

double calculateIndivualEnergy(cv::Mat &in_image, int i, int j);

double gaussianProb(double en, double mean, double var);

void contactGraphWithMainSinkSource(Graph &graph, int rows, int cols, double fg_mean, double bg_mean, double fg_var, double bg_var, std::vector<std::vector<double> > energy,Area foreground ,Area background);

void intializePixelAllocation(int cols, int rows, cv::Mat &input, Area foreground, Area background, Graph &graph);

cv::Mat createGraph(cv::Mat &output, cv::Mat &input, Area foreground, Area background);

#endif
