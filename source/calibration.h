/*
Copyright (c) 2015 C. D. Tharindu Mathew
http://mackiemathew.wordpress.com

This project is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <http://www.gnu.org/licenses/agpl-3.0.html>.
*/
#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "opencv2/core/core.hpp"
#include "crosshairs.h"

class Calibration
{
private:
	std::vector <glm::dvec3> worldPts1;
	std::vector <glm::dvec2> imgPts1;
	std::vector <glm::dvec3> worldPts2;
	std::vector <glm::dvec2> imgPts2;
	std::vector <glm::dvec3> worldPts3;
	std::vector <glm::dvec2> imgPts3;
	cv::Mat h1; 
	cv::Mat h2;

	cv::Mat A;
	cv::Mat Rt;
	double lambda;

	double k1;
	double k2;

	cv::Mat E1;
	cv::Mat E2;
	cv::Mat E3;

	void readInputFile(std::string fileName, std::vector <glm::dvec3>& worldPts, std::vector <glm::dvec2>& imgPts, bool denormalize = true);
	void writeFile(std::string fileName, std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts);
	cv::Mat constructV(int i, int j, cv::Mat& h);

public:
	void posCrossHairs(CrossHairs* crossHairs, int imgNo);
	void calcIntrinsicMat(cv::Mat& h1, cv::Mat &h2, cv::Mat& A, double& lambda);
	void calcExtrinsicMat(cv::Mat& A, cv::Mat& h, cv::Mat& E);
	void calcRMSE(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat&A, cv::Mat&E);
	void calcRMSE(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat&A, cv::Mat&E, double& k1, double& k2);
	void calcRMSE(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat&h);
	void calcHomography(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat& h1);
	void saveImgPnts(std::vector <glm::dvec3> worldPts, std::vector <glm::dvec2> imgPts, int imgNo);
	void load_img_points(CrossHairs* crossHairs, int imgNo);
	void calibrate();
	inline cv::Mat getE1() { return E1; };
	inline cv::Mat getE2() { return E2; };
	inline cv::Mat getE3() { return E3; };
	inline cv::Mat getA() { return A; };
	Calibration();
	~Calibration();
};

