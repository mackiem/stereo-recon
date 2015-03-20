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
#include "calibration.h"
#include <fstream>
#include <sstream>
#include <string>
#include <cassert>
#include <iostream>

static const double w = 1024.0;
static const double h = w;

typedef std::vector<glm::dvec3> WPts;
typedef std::vector<glm::dvec2> IPts;

void Calibration::calcHomography(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat& h) {
	const int rows = worldPts.size() * 2;
	cv::Mat A(rows, 9, CV_64F);
	for (int i = 0; i < worldPts.size(); ++i) {
		for (int j = 0; j < 2; ++j) {
		    A.at<double>(2 * i, j) = worldPts[i][j];
		    A.at<double>((2 * i) + 1, j) = 0.0;
		}
		A.at<double>(2 * i, 2) = 1.0;
		A.at<double>((2 * i) + 1, 2) = 0.0;

		for (int j = 3; j < 5; ++j) {
		    A.at<double>((2 * i), j) = 0.0;
		    A.at<double>((2 * i) + 1, j) = worldPts[i][j-3];
		}
		A.at<double>((2 * i), 5) = 0.0;
		A.at<double>((2 * i) + 1, 5) = 1.0;

		for (int j = 6; j < 8; ++j) {
		    A.at<double>(2 * i, j) = -1 * imgPts[i][0] * worldPts[i][j-6];
		    A.at<double>((2 * i) + 1, j) = -1 * imgPts[i][1] * worldPts[i][j-6];
		}
		A.at<double>(2 * i, 8) = -1 * imgPts[i][0];
		A.at<double>((2 * i) + 1, 8) = -1 * imgPts[i][1];
	}
	std::ofstream file;
	file.open("mat.txt");
	//std::cout << cv::format(A, "matlab") << std::endl;
	//file << cv::format(A, "matlab") << std::endl;
	file.close();
	cv::Mat w, u, vt;
	cv::SVD::compute(A, w, u, vt);
	//std::cout << vt << std::endl;

	h = cv::Mat(3, 3, CV_64F);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			h.at<double>(i, j) = (vt.at<double>(vt.rows - 1, i * 3 + j));
		}
	}
	//std::cout << h << std::endl;

}

void Calibration::posCrossHairs(CrossHairs* crossHairs, int imgNo)  {
	auto& map = crossHairs->pos;
	std::vector <glm::dvec3> worldPts;
	std::vector <glm::dvec2> imgPts;
	cv::Mat E;
	if (imgNo == 1) {
		worldPts = worldPts1;
		imgPts = imgPts1;
		E = E1;
	}
	else if (imgNo == 2) {
		worldPts = worldPts2;
		imgPts = imgPts2;
		E = E2;
	} else if (imgNo == 3) {
		worldPts = worldPts3;
		imgPts = imgPts3;
		E = E3;
	}
    
	for (int i = 0; i < worldPts.size(); ++i) {
		cv::Mat pt(4, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = worldPts[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		pt.at<double>(3, 0) = 1.0;
		cv::Mat result = A * E * pt;

		double u = (result.at<double>(0, 0) / result.at<double>(2, 0));
		double v = (result.at<double>(1, 0) / result.at<double>(2, 0));
		double x = u / w;
		double y = v / w;

		double u0 = A.at<double>(0, 2);
		double v0 = A.at<double>(1, 2);

		double lSq = std::pow(x, 2) + std::pow(y, 2);
		double uHat = u + ((u - u0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double vHat = v + ((v - v0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		map[i] = glm::vec3((uHat / w) -1, (vHat / h) - 1, 0.f);
	    //std::cout << result << std::endl;
	    //std::cout << result.at<double>(0, 0) / result.at<double>(2, 0) << ", " << result.at<double>(1, 0) / result.at<double>(2, 0) << std::endl;
	    //std::cout << result.at<double>(0, 0) << ", " << result.at<double>(1, 0) << std::endl;
	}
}

void writeMatrixToFile(std::string fileName, cv::Mat& A) {
	std::ofstream file;
	file.open(fileName);
	file << cv::format(A, "matlab") << std::endl;
	file.close();
}

void Calibration::load_img_points(CrossHairs* crossHairs, int imgNo) {
    
	WPts world_pts;
	IPts img_pts;
	if (imgNo == 1) {
		readInputFile(std::string("corr1.cp"), world_pts, img_pts, false);
	}
	else if (imgNo == 2) {
		readInputFile(std::string("corr2.cp"), world_pts, img_pts, false);
	}
	else if (imgNo == 3) {
		readInputFile(std::string("corr3.cp"), world_pts, img_pts, false);
	}
	for (auto i = 0u; i < img_pts.size(); ++i) {
		crossHairs->adjust(i, img_pts[i].x, img_pts[i].y);
	}
    
    
}

cv::Mat Calibration::constructV(int i, int j, cv::Mat& h) {
	cv::Mat mat(1, 6, CV_64F);
	mat.at<double>(0, 0) = h.at<double>(i, 0) * h.at<double>(j, 0);
	mat.at<double>(0, 1) = h.at<double>(i, 0) * h.at<double>(j, 1) + h.at<double>(i, 1) * h.at<double>(j, 0);
	mat.at<double>(0, 2) = h.at<double>(i, 1) * h.at<double>(j, 1);
	mat.at<double>(0, 3) = h.at<double>(i, 2) * h.at<double>(j, 0) + h.at<double>(i, 0) * h.at<double>(j, 2);
	mat.at<double>(0, 4) = h.at<double>(i, 2) * h.at<double>(j, 1) + h.at<double>(i, 1) * h.at<double>(j, 2);
	mat.at<double>(0, 5) = h.at<double>(i, 2) * h.at<double>(j, 2);
	return mat.t();
}

void Calibration::calcRMSE(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat&h) {
	double xtotal = 0.0;
	double ytotal = 0.0;
	for (int i = 0; i < worldPts.size(); ++i) {
		cv::Mat pt(3, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = worldPts[i][x];
		}
		//pt.at<double>(2, 0) = 0.0;
		pt.at<double>(2, 0) = 1.0;
		cv::Mat result = h * pt;
		xtotal += std::pow(imgPts[i].x - (result.at<double>(0, 0)/result.at<double>(2, 0)), 2);
		ytotal += std::pow(imgPts[i].y - (result.at<double>(1, 0)/result.at<double>(2, 0)), 2);
	    //std::cout << result << std::endl;
	    //std::cout << ((result.at<double>(0, 0) / result.at<double>(2, 0)) / 1024.0) - 1 << ", " << ((result.at<double>(1, 0) / result.at<double>(2, 0)) / 1024.0) - 1 << std::endl;
	    //std::cout << result.at<double>(0, 0) << ", " << result.at<double>(1, 0) << std::endl;
	}
	std::cout << "RMSE : (" << std::sqrt(xtotal / (double)worldPts.size()) << ", " << std::sqrt(ytotal / (double)worldPts.size()) << ")" << std::endl;
}

void Calibration::calcRMSE(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat&A, cv::Mat&E) {
	double xtotal = 0.0;
	double ytotal = 0.0;
	for (int i = 0; i < worldPts.size(); ++i) {
		cv::Mat pt(4, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = worldPts[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		pt.at<double>(3, 0) = 1.0;
		cv::Mat result = A * E * pt;
		xtotal += std::pow(imgPts[i].x - (result.at<double>(0, 0) / result.at<double>(2, 0)), 2);
		ytotal += std::pow(imgPts[i].y - (result.at<double>(1, 0) / result.at<double>(2, 0)), 2);
	    //std::cout << result << std::endl;
	    //std::cout << result.at<double>(0, 0) / result.at<double>(2, 0) << ", " << result.at<double>(1, 0) / result.at<double>(2, 0) << std::endl;
	    //std::cout << result.at<double>(0, 0) << ", " << result.at<double>(1, 0) << std::endl;
	}
	std::cout << "RMSE : (" << std::sqrt(xtotal / (double)worldPts.size()) << ", " << std::sqrt(ytotal / (double)worldPts.size()) << ")" << std::endl;
	//std::cout << A << E << std::endl;
}

void Calibration::calcRMSE(std::vector<glm::dvec3>& worldPts, std::vector<glm::dvec2>& imgPts, cv::Mat&A, cv::Mat&E, double& k1, double& k2) {
	double xtotal = 0.0;
	double ytotal = 0.0;
	for (int i = 0; i < worldPts.size(); ++i) {
		cv::Mat pt(4, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = worldPts[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		pt.at<double>(3, 0) = 1.0;
		cv::Mat result = A * E * pt;

		double u = (result.at<double>(0, 0) / result.at<double>(2, 0));
		double v = (result.at<double>(1, 0) / result.at<double>(2, 0));
		double x = u / w;
		double y = v / w;

		double u0 = A.at<double>(0, 2);
		double v0 = A.at<double>(1, 2);

		double lSq = std::pow(x, 2) + std::pow(y, 2);
		double uHat = u + ((u - u0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double vHat = v + ((v - v0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));

		xtotal += std::pow(imgPts[i].x - (uHat), 2);
		ytotal += std::pow(imgPts[i].y - (vHat), 2);
	    //std::cout << result << std::endl;
	    //std::cout << result.at<double>(0, 0) / result.at<double>(2, 0) << ", " << result.at<double>(1, 0) / result.at<double>(2, 0) << std::endl;
	    //std::cout << result.at<double>(0, 0) << ", " << result.at<double>(1, 0) << std::endl;
	}
	std::cout << "RMSE : (" << std::sqrt(xtotal / (double)worldPts.size()) << ", " << std::sqrt(ytotal / (double)worldPts.size()) << ")" << std::endl;

}

void Calibration::calcExtrinsicMat(cv::Mat& A, cv::Mat& h, cv::Mat& E) {
	cv::Mat invA = A.inv();
	cv::Mat tmp = (invA * h.col(0));
	double lambda = 1.0 / cv::norm(tmp);
	cv::Mat r1 = (invA * h.col(0));
	r1 *= lambda;

	cv::Mat r2 = (invA * h.col(1));
	r2 *= lambda;

	cv::Mat r3 = r1.cross(r2);

	cv::Mat t = (invA * h.col(2));
	t *= lambda;

	E = cv::Mat(3, 4, CV_64F);
	r1.col(0).copyTo(E.col(0));
	r2.col(0).copyTo(E.col(1));
	r3.col(0).copyTo(E.col(2));
	t.col(0).copyTo(E.col(3));

}

void Calibration::calcIntrinsicMat(cv::Mat& h1, cv::Mat &h2, cv::Mat& A, double& lambda) {
	cv::Mat v(7, 6, CV_64F);

	cv::Mat r1 = constructV(0, 1, h1).t();
	r1.row(0).copyTo(v.row(0));
	cv::Mat tmp = (constructV(0, 0, h1) - constructV(1, 1, h1)).t();
    tmp.row(0).copyTo(v.row(1));
    
	cv::Mat r2 = constructV(0, 1, h2).t();
	r1.row(0).copyTo(v.row(2));
	cv::Mat tmp2 = (constructV(0, 0, h2) - constructV(1, 1, h2)).t();
    tmp2.row(0).copyTo(v.row(3));

    // Assuming gamma, u0, v0 is mid point
	double u0 = w/2.0;
	double v0 = h/2.0;

	double gamm0[1][6] = { { 0, 1, 0, 0, 0, 0 } };
	cv::Mat(1, 6, CV_64F, gamm0).copyTo(v.row(4));

	double u0v01[1][6] = { { u0, 0, 0, 1, 0, 0} };
	cv::Mat(1, 6, CV_64F, u0v01).copyTo(v.row(5));

	double u0v02[1][6] = { { v0, 0, 0, 0, 1, 0} };
	cv::Mat(1, 6, CV_64F, u0v02).copyTo(v.row(6));

	//std::cout << v << std::endl;
	writeMatrixToFile("mat.txt", v);


	cv::Mat w, u, vt;
	cv::SVD::compute(v, w, u, vt);
	//std::cout << vt << std::endl;

	//cv::Mat B = cv::Mat(1, 6, CV_64F);
	cv::Mat b = vt.row(vt.rows - 1).t();
	//std::cout << b << std::endl;

    // b = [ B11,  B12, B22, B13, B23, B33 ]^T
	//double v0 = (b.at<double>(1, 0) * b.at<double>(3, 0) - b.at<double>(0, 0) * b.at<double>(4, 0)) / (b.at<double>(0, 0) * b.at<double>(2, 0) - std::pow(b.at<double>(1, 0), 2));
	lambda = b.at<double>(5, 0) - ((std::pow(b.at<double>(3, 0), 2) + v0 * ((b.at<double>(1, 0) * b.at<double>(3, 0) - b.at<double>(0, 0) * b.at<double>(4, 0)))) / b.at<double>(0, 0));
	//lambda = std::abs(lambda);
	double alpha = cv::sqrt(std::abs(lambda / b.at<double>(0, 0)));
	double beta = std::sqrt(std::abs(lambda * b.at<double>(0, 0) / (b.at<double>(0, 0) * b.at<double>(2, 0) - std::pow(b.at<double>(1, 0), 2))) );
	double gamma = 0.0;

	A = cv::Mat ( 3, 3, CV_64F );
	A.at<double>(0, 0) = alpha;
	A.at<double>(0, 1) = gamma;
	A.at<double>(0, 2) = u0;
	A.at<double>(1, 0) = 0.0;
	A.at<double>(1, 1) = beta;
	A.at<double>(1, 2) = v0;
	A.at<double>(2, 0) = 0.0;
	A.at<double>(2, 1) = 0.0;
	A.at<double>(2, 2) = 1.0;

	//std::cout << A << std::endl;
}



extern "C" int mylmdif_(int (*fcn)(int *, int *, double *, double *, int *), int *m, int *n, double *x, double *fvec, double *ftol, double *xtol, double *gtol, int *maxfev, 
	double *epsfcn, double *diag, int *mode, double *factor, int *nprint, int *info, int *nfev, double *fjac, int *ldfjac, int *ipvt, 
	double *qtf, double *wa1, double *wa2, double *wa3, double *wa4);
//
//static std::vector <glm::vec3> _worldPts;
//static std::vector <glm::vec2> _imgPts;
//
///*****************************************************************************
//*****************************************************************************/

//static int imgNo;
	std::vector<glm::dvec3> *gWorldPts1;
	std::vector<glm::dvec2> *gImgPts1;
	std::vector<glm::dvec3> *gWorldPts2;
	std::vector<glm::dvec2> *gImgPts2;
	std::vector<glm::dvec3> *gWorldPts3;
	std::vector<glm::dvec2> *gImgPts3;

static int
lmdifError_(int *m_ptr, int *n_ptr, double *params, double *error, int *)
{
	int nparms = *n_ptr;
	int nerrors = *m_ptr;
	//  array = {a, b, r11, r12, r13, r21, r22, r23, r31, r32, r33, t11, t21, t23, k1, k2}
	//double rArr[9];
	//std::copy(parms + 2, parms + 11, rArr);
	//double tArr[3];
	//std::copy(parms + 11, parms + 14, tArr);

	//cv::Mat R(3, 3, CV_64F, rArr);
	//cv::Mat t(3, 1, CV_64F, tArr);



	//double k1 = parms[14];
	//double k2 = parms[15];

	//double u0 = w / 2.0;
	//double v0 = h / 2.0;

	//cv::Mat A(3, 3, CV_64F);
	//A.at<double>(0, 0) = parms[0];
	//A.at<double>(0, 1) = 0;
	//A.at<double>(0, 2) = u0;
	//A.at<double>(1, 0) = 0.0;
	//A.at<double>(1, 1) = parms[1];
	//A.at<double>(1, 2) = v0;
	//A.at<double>(2, 0) = 0.0;
	//A.at<double>(2, 1) = 0.0;
	//A.at<double>(2, 2) = 1.0;

	double k1 = params[38];
	double k2 = params[39];

	double u0 = w / 2.0;
	double v0 = h / 2.0;


	cv::Mat A(3, 3, CV_64F);
	A.at<double>(0, 0) = params[0];
	A.at<double>(0, 1) = 0;
	A.at<double>(0, 2) = u0;
	A.at<double>(1, 0) = 0.0;
	A.at<double>(1, 1) = params[1];
	A.at<double>(1, 2) = v0;
	A.at<double>(2, 0) = 0.0;
	A.at<double>(2, 1) = 0.0;
	A.at<double>(2, 2) = 1.0;

	double rArr[9];
	std::copy(params + 2, params + 11, rArr);
	double tArr[3];
	std::copy(params + 11, params + 14, tArr);

	cv::Mat R1(3, 3, CV_64F, rArr);
	cv::Mat t1(3, 1, CV_64F, tArr);


	std::copy(params + 14, params + 23, rArr);
	std::copy(params + 23, params + 26, tArr);

	cv::Mat R2(3, 3, CV_64F, rArr);
	cv::Mat t2(3, 1, CV_64F, tArr);


	std::copy(params + 26, params + 35, rArr);
	std::copy(params + 35, params + 38, tArr);

	cv::Mat R3(3, 3, CV_64F, rArr);
	cv::Mat t3(3, 1, CV_64F, tArr);

	//   double xtotal = 0.0;
	//double ytotal = 0.0;
	// calc error
	for (int i = 0; i < gWorldPts1->size(); ++i) {
		cv::Mat pt(3, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = (*gWorldPts1)[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		//pt.at<double>(3, 0) = 1.0;
		cv::Mat tmp = R1 * pt;
		tmp += t1;
		tmp = A * tmp;
		double u = (tmp.at<double>(0, 0) / tmp.at<double>(2, 0));
		double v = (tmp.at<double>(1, 0) / tmp.at<double>(2, 0));
		double x = u / w;
		double y = v / w;
		double lSq = std::pow(x, 2) + std::pow(y, 2);
		double uHat = u + ((u - u0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double vHat = v + ((v - v0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double xtotal = std::pow(((*gImgPts1)[i].x - uHat), 2);
		double ytotal = std::pow(((*gImgPts1)[i].y - vHat), 2);
		//error[i] = std::sqrt(xtotal + ytotal);
		error[i] = std::pow((*gImgPts1)[i].x - uHat, 2) + std::pow((*gImgPts1)[i].y - vHat, 2);
	}

	for (int i = 0; i < (*gWorldPts2).size(); ++i) {
		cv::Mat pt(3, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = (*gWorldPts2)[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		//pt.at<double>(3, 0) = 1.0;
		cv::Mat tmp = R2 * pt;
		tmp += t2;
		tmp = A * tmp;
		double u = (tmp.at<double>(0, 0) / tmp.at<double>(2, 0));
		double v = (tmp.at<double>(1, 0) / tmp.at<double>(2, 0));
		double x = u / w;
		double y = v / w;
		double lSq = std::pow(x, 2) + std::pow(y, 2);
		double uHat = u + ((u - u0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double vHat = v + ((v - v0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double xtotal = std::pow(((*gImgPts2)[i].x - uHat), 2);
		double ytotal = std::pow(((*gImgPts2)[i].y - vHat), 2);
		//error[i] = std::sqrt(xtotal + ytotal);
		error[(*gWorldPts1).size() + i] = std::pow((*gImgPts2)[i].x - uHat, 2) + std::pow((*gImgPts2)[i].y - vHat, 2);
	}

	for (int i = 0; i < (*gWorldPts3).size(); ++i) {
		cv::Mat pt(3, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = (*gWorldPts3)[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		//pt.at<double>(3, 0) = 1.0;
		cv::Mat tmp = R3 * pt;
		tmp += t3;
		tmp = A * tmp;
		double u = (tmp.at<double>(0, 0) / tmp.at<double>(2, 0));
		double v = (tmp.at<double>(1, 0) / tmp.at<double>(2, 0));
		double x = u / w;
		double y = v / w;
		double lSq = std::pow(x, 2) + std::pow(y, 2);
		double uHat = u + ((u - u0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double vHat = v + ((v - v0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double xtotal = std::pow(((*gImgPts3)[i].x - uHat), 2);
		double ytotal = std::pow(((*gImgPts3)[i].y - vHat), 2);
		//error[i] = std::sqrt(xtotal + ytotal);
		error[(*gWorldPts1).size() + (*gWorldPts2).size() + i] = std::pow((*gImgPts3)[i].x - uHat, 2) + std::pow((*gImgPts3)[i].y - vHat, 2);
	}
	return 1;
}
static int
lmdifError(int *m_ptr, int *n_ptr, double *params, double *error, int *)
{
	int nparms = *n_ptr;
	int nerrors = *m_ptr;
	//  array = {a, b, r11, r12, r13, r21, r22, r23, r31, r32, r33, t11, t21, t23, k1, k2}
	//double rArr[9];
	//std::copy(parms + 2, parms + 11, rArr);
	//double tArr[3];
	//std::copy(parms + 11, parms + 14, tArr);

	//cv::Mat R(3, 3, CV_64F, rArr);
	//cv::Mat t(3, 1, CV_64F, tArr);



	//double k1 = parms[14];
	//double k2 = parms[15];

	//double u0 = w / 2.0;
	//double v0 = h / 2.0;

	//cv::Mat A(3, 3, CV_64F);
	//A.at<double>(0, 0) = parms[0];
	//A.at<double>(0, 1) = 0;
	//A.at<double>(0, 2) = u0;
	//A.at<double>(1, 0) = 0.0;
	//A.at<double>(1, 1) = parms[1];
	//A.at<double>(1, 2) = v0;
	//A.at<double>(2, 0) = 0.0;
	//A.at<double>(2, 1) = 0.0;
	//A.at<double>(2, 2) = 1.0;

	double k1 = params[14];
	double k2 = params[15];

	double u0 = w / 2.0;
	double v0 = h / 2.0;


	cv::Mat A(3, 3, CV_64F);
	A.at<double>(0, 0) = params[0];
	A.at<double>(0, 1) = 0;
	A.at<double>(0, 2) = u0;
	A.at<double>(1, 0) = 0.0;
	A.at<double>(1, 1) = params[1];
	A.at<double>(1, 2) = v0;
	A.at<double>(2, 0) = 0.0;
	A.at<double>(2, 1) = 0.0;
	A.at<double>(2, 2) = 1.0;

	double rArr[9];
	std::copy(params + 2, params + 11, rArr);
	double tArr[3];
	std::copy(params + 11, params + 14, tArr);

	cv::Mat R1(3, 3, CV_64F, rArr);
	cv::Mat t1(3, 1, CV_64F, tArr);


	//   double xtotal = 0.0;
	//double ytotal = 0.0;
	// calc error
	for (int i = 0; i < gWorldPts1->size(); ++i) {
		cv::Mat pt(3, 1, CV_64F);
		for (int x = 0; x < 2; ++x) {
			pt.at<double>(x, 0) = (*gWorldPts1)[i][x];
		}
		pt.at<double>(2, 0) = 0.0;
		//pt.at<double>(3, 0) = 1.0;
		cv::Mat tmp = R1 * pt;
		tmp += t1;
		tmp = A * tmp;
		double u = (tmp.at<double>(0, 0) / tmp.at<double>(2, 0));
		double v = (tmp.at<double>(1, 0) / tmp.at<double>(2, 0));
		double x = u / w;
		double y = v / w;
		double lSq = std::pow(x, 2) + std::pow(y, 2);
		double uHat = u + ((u - u0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double vHat = v + ((v - v0) * ((k1 * lSq) + (k2 * std::pow(lSq, 2))));
		double xtotal = std::pow(((*gImgPts1)[i].x - uHat), 2);
		double ytotal = std::pow(((*gImgPts1)[i].y - vHat), 2);
		//error[i] = std::sqrt(xtotal + ytotal);
		error[i] = std::pow((*gImgPts1)[i].x - uHat, 2) + std::pow((*gImgPts1)[i].y - vHat, 2);
	}

	return 1;
}

 //  float r11 = parms[0];
 //  float r12 = parms[1];
 //  float tx = parms[2];
 //  float r21 = parms[3];
 //  float r22 = parms[4];
 //  float ty = 1;

	//assert(nparms == 5);
	//assert(nerrors == _worldPts.size());

	//float m = parms[0];
	//float b = parms[1];
	//assert(nparms == 2);
	//assert(nerrors == _gradPt.size());

	//// compute error terms
 //  for (int i=0; i<_gradPt.size(); i++) {
	//	VM2Point p = _gradPt[i];
	//	VM2Point s;
	//	if (_horizontalLine == TRUE) {
	//		VM2Point q(0.0f, b);
	//		VM2Point r(1.0f, m+b);
	//		VM2Vector v = r-q;
	//		float t = (p.x() - q.x()) / v.x();
	//		s = q + t*VM2Point(v);
	//	} else {
	//		VM2Point q(b,   0.0f);
	//		VM2Point r(m+b, 1.0f);
	//		VM2Vector v = r-q;
	//		float t = (p.y() - q.y()) / v.y();
	//		s = q + t*VM2Point(v);
	//	}
	//	VM2Vector diff = p-s;
	//	float distSqr = diff.lengthSquared();
	//	error[i] = distSqr;
 //  }

 //  return (TRUE);
//}
//
//

/*****************************************************************************
*****************************************************************************/
/* Parameters controlling MINPACK's lmdif() optimization routine. */
/* See the file lmdif.f for definitions of each parameter.        */
#define REL_SENSOR_TOLERANCE_ftol    1.0E-6      /* [pix] */
#define REL_PARAM_TOLERANCE_xtol     1.0E-7
#define ORTHO_TOLERANCE_gtol         0.0
#define MAXFEV                       (1000*n)
#define EPSFCN                       1.0E-6 /* was E-16 Do not set to 0! */
#define MODE                         2       /* variables scaled internally */
#define FACTOR                       100.0 
static int
optCalib(WPts& worldPts, IPts& imgPts, cv::Mat& A, cv::Mat& E1, double& k1, double& k2)
{
    /* Parameters needed by MINPACK's lmdif() */
	int     n = 16;
	int     m = worldPts.size();
    double *x;
    double *fvec;
    double  ftol = REL_SENSOR_TOLERANCE_ftol;
    double  xtol = REL_PARAM_TOLERANCE_xtol;
    double  gtol = ORTHO_TOLERANCE_gtol;
    int     maxfev = MAXFEV;
    double  epsfcn = EPSFCN;
    double *diag;
    int     mode = MODE;
    double  factor = FACTOR;
    int     ldfjac = m;
    int     nprint = 0;
    int     info;
    int     nfev;
    double *fjac;
    int    *ipvt;
    double *qtf;
    double *wa1;
    double *wa2;
    double *wa3;
    double *wa4;
	 double worldSize = w;

	//  array = {a, b, r11, r12, r13, r21, r22, r23, r31, r32, r33, t11, t21, t23, k1, k2}
	 const int num = 40;
	 double params[num];

	 params[0] = A.at<double>(0, 0);
	 params[1] = A.at<double>(1, 1);

     // R matrix
	 for (int i = 0; i < 3; ++i) {
		 for (int j = 0; j < 3; ++j) {
			 params[2 + (3 * i + j) ] = E1.at<double>(i, j);
		 }
	 }

	 for (int j = 0; j < 3; ++j) {
		 params[11 + j] = E1.at<double>(j, 3);
	 }


     // init k1, k2 to 0 as guesses
	 params[14] = 0.0;
	 params[15] = 0.0;

	 //std::cout << A << std::endl;
	 //std::cout << E << std::endl;





	 /* copy to globals */
	 gWorldPts1 = &worldPts;
	 gImgPts1 = &imgPts;

    /* allocate stuff dependent on n */
    x    = (double *)calloc(n, sizeof(double));
    diag = (double *)calloc(n, sizeof(double));
    qtf  = (double *)calloc(n, sizeof(double));
    wa1  = (double *)calloc(n, sizeof(double));
    wa2  = (double *)calloc(n, sizeof(double));
    wa3  = (double *)calloc(n, sizeof(double));
    ipvt = (int    *)calloc(n, sizeof(int));

    /* allocate some workspace */
    if (( fvec = (double *) calloc ((unsigned int) m, 
                                    (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace fvec\n");
       exit(-1);
    }

    if (( fjac = (double *) calloc ((unsigned int) m*n,
                                    (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace fjac\n");
       exit(-1);
    }

    if (( wa4 = (double *) calloc ((unsigned int) m, 
                                   (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace wa4\n");
       exit(-1);
    }


    /* copy parameters in as initial values */
	//std::cout << "initial val. :";
	for (int i = 0; i < n; i++) {
       x[i] = params[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;

    /* define optional scale factors for the parameters */
    if ( mode == 2 ) {
		int offset = 0;
		for (int offset = 0; offset<n; offset++) {
			diag[offset] = 1.0;
		}
    }

    /* perform the optimization */ 
    //printf("Starting optimization step...\n");
    mylmdif_ (lmdifError,
            &m, &n, x, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn,
            diag, &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac,
            ipvt, qtf, wa1, wa2, wa3, wa4);
    double totalerror = 0;
    for (int i=0; i<m; i++)
       totalerror += fvec[i];
    printf("\tnum function calls = %i\n", nfev);
    printf("\tremaining total error value = %f\n", totalerror);
    printf("\tor %1.2f per point\n", std::sqrt(totalerror) / m);
    printf("...ended optimization step.\n");

    /* copy result back to parameters array */
	//std::cout << "final val. :";
    for (int i=0; i<n; i++) {
       params[i] = x[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;

	//  array = {a, b, r11, r12, r13, r21, r22, r23, r31, r32, r33, t11, t21, t23, k1, k2}


	k1 = params[14];
	k2 = params[15];

	double u0 = w / 2.0;
	double v0 = h / 2.0;


	A.at<double>(0, 0) = params[0];
	A.at<double>(0, 1) = 0;
	A.at<double>(0, 2) = u0;
	A.at<double>(1, 0) = 0.0;
	A.at<double>(1, 1) = params[1];
	A.at<double>(1, 2) = v0;
	A.at<double>(2, 0) = 0.0;
	A.at<double>(2, 1) = 0.0;
	A.at<double>(2, 2) = 1.0;

	double rArr[9];
	std::copy(params + 2, params + 11, rArr);
	double tArr[3];
	std::copy(params + 11, params + 14, tArr);

	cv::Mat R1(3, 3, CV_64F, rArr);
	cv::Mat t1(3, 1, CV_64F, tArr);

	R1.col(0).copyTo(E1.col(0));
	R1.col(1).copyTo(E1.col(1));
	R1.col(2).copyTo(E1.col(2));
	t1.col(0).copyTo(E1.col(3));

	std::cout << "Final Results: " << std::endl;
	std::cout << "A: " << A << std::endl;
	std::cout << "External 1: " << E1 << std::endl;
	std::cout << "k1 : " << k1 << std::endl;
	std::cout << "k2 : " << k2 << std::endl;

    /* release allocated workspace */
    free (fvec);
    free (fjac);
    free (wa4);
    free (ipvt);
    free (wa1);
    free (wa2);
    free (wa3);
    free (qtf);
    free (diag);
    free (x);

	 return (1);
}

static int
optCalib_(WPts& worldPts1, IPts& imgPts1, WPts& worldPts2, IPts& imgPts2, WPts& worldPts3, IPts& imgPts3, cv::Mat& A, cv::Mat& E1, cv::Mat &E2, cv::Mat& E3, double& k1, double& k2)
{
    /* Parameters needed by MINPACK's lmdif() */
	int     n = 16 + 24;
    int     m = worldPts1.size() + worldPts2.size() + worldPts3.size();
    double *x;
    double *fvec;
    double  ftol = REL_SENSOR_TOLERANCE_ftol;
    double  xtol = REL_PARAM_TOLERANCE_xtol;
    double  gtol = ORTHO_TOLERANCE_gtol;
    int     maxfev = MAXFEV;
    double  epsfcn = EPSFCN;
    double *diag;
    int     mode = MODE;
    double  factor = FACTOR;
    int     ldfjac = m;
    int     nprint = 0;
    int     info;
    int     nfev;
    double *fjac;
    int    *ipvt;
    double *qtf;
    double *wa1;
    double *wa2;
    double *wa3;
    double *wa4;
	 double worldSize = w;

	//  array = {a, b, r11, r12, r13, r21, r22, r23, r31, r32, r33, t11, t21, t23, k1, k2}
	 const int num = 40;
	 double params[num];

	 params[0] = A.at<double>(0, 0);
	 params[1] = A.at<double>(1, 1);

     // R matrix
	 for (int i = 0; i < 3; ++i) {
		 for (int j = 0; j < 3; ++j) {
			 params[2 + (3 * i + j) ] = E1.at<double>(i, j);
		 }
	 }

	 for (int j = 0; j < 3; ++j) {
		 params[11 + j] = E1.at<double>(j, 3);
	 }

     // R matrix
	 for (int i = 0; i < 3; ++i) {
		 for (int j = 0; j < 3; ++j) {
			 params[14 + (3 * i + j) ] = E2.at<double>(i, j);
		 }
	 }

	 for (int j = 0; j < 3; ++j) {
		 params[23 + j] = E2.at<double>(j, 3);
	 }

	 for (int i = 0; i < 3; ++i) {
		 for (int j = 0; j < 3; ++j) {
			 params[26 + (3 * i + j) ] = E3.at<double>(i, j);
		 }
	 }

	 for (int j = 0; j < 3; ++j) {
		 params[35 + j] = E3.at<double>(j, 3);
	 }

     // init k1, k2 to 0 as guesses
	 params[38] = 0.0;
	 params[39] = 0.0;

	 //std::cout << A << std::endl;
	 //std::cout << E << std::endl;





	 /* copy to globals */
	 gWorldPts1 = &worldPts1;
	 gImgPts1 = &imgPts1;
	 gWorldPts2 = &worldPts2;
	 gImgPts2 = &imgPts2;
	 gWorldPts3 = &worldPts3;
	 gImgPts3 = &imgPts3;

    /* allocate stuff dependent on n */
    x    = (double *)calloc(n, sizeof(double));
    diag = (double *)calloc(n, sizeof(double));
    qtf  = (double *)calloc(n, sizeof(double));
    wa1  = (double *)calloc(n, sizeof(double));
    wa2  = (double *)calloc(n, sizeof(double));
    wa3  = (double *)calloc(n, sizeof(double));
    ipvt = (int    *)calloc(n, sizeof(int));

	ldfjac = m;
    /* allocate some workspace */
    if (( fvec = (double *) calloc ((unsigned int) m, 
                                    (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace fvec\n");
       exit(-1);
    }

    if (( fjac = (double *) calloc ((unsigned int) m*n,
                                    (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace fjac\n");
       exit(-1);
    }

    if (( wa4 = (double *) calloc ((unsigned int) m, 
                                   (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace wa4\n");
       exit(-1);
    }


    /* copy parameters in as initial values */
	//std::cout << "initial val. :";
	for (int i = 0; i < n; i++) {
       x[i] = params[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;

    /* define optional scale factors for the parameters */
    if ( mode == 2 ) {
		int offset = 0;
		for (int offset = 0; offset<n; offset++) {
			diag[offset] = 1.0;
		}
    }

    /* perform the optimization */ 
    //printf("Starting optimization step...\n");
    mylmdif_ (lmdifError_,
            &m, &n, x, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn,
            diag, &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac,
            ipvt, qtf, wa1, wa2, wa3, wa4);
    double totalerror = 0;
    for (int i=0; i<m; i++)
       totalerror += fvec[i];
    printf("\tnum function calls = %i\n", nfev);
    printf("\tremaining total error value = %f\n", totalerror);
    printf("\tor %1.2f per point\n", std::sqrt(totalerror) / m);
    printf("...ended optimization step.\n");

    /* copy result back to parameters array */
	//std::cout << "final val. :";
    for (int i=0; i<n; i++) {
       params[i] = x[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;

	//  array = {a, b, r11, r12, r13, r21, r22, r23, r31, r32, r33, t11, t21, t23, k1, k2}


	k1 = params[38];
	k2 = params[39];

	double u0 = w / 2.0;
	double v0 = h / 2.0;


	A.at<double>(0, 0) = params[0];
	A.at<double>(0, 1) = 0;
	A.at<double>(0, 2) = u0;
	A.at<double>(1, 0) = 0.0;
	A.at<double>(1, 1) = params[1];
	A.at<double>(1, 2) = v0;
	A.at<double>(2, 0) = 0.0;
	A.at<double>(2, 1) = 0.0;
	A.at<double>(2, 2) = 1.0;

	double rArr[9];
	std::copy(params + 2, params + 11, rArr);
	double tArr[3];
	std::copy(params + 11, params + 14, tArr);

	cv::Mat R1(3, 3, CV_64F, rArr);
	cv::Mat t1(3, 1, CV_64F, tArr);

	R1.col(0).copyTo(E1.col(0));
	R1.col(1).copyTo(E1.col(1));
	R1.col(2).copyTo(E1.col(2));
	t1.col(0).copyTo(E1.col(3));

	std::copy(params + 14, params + 23, rArr);
	std::copy(params + 23, params + 26, tArr);

	cv::Mat R2(3, 3, CV_64F, rArr);
	cv::Mat t2(3, 1, CV_64F, tArr);

	R2.col(0).copyTo(E2.col(0));
	R2.col(1).copyTo(E2.col(1));
	R2.col(2).copyTo(E2.col(2));
	t2.col(0).copyTo(E2.col(3));

	std::copy(params + 26, params + 35, rArr);
	std::copy(params + 35, params + 38, tArr);

	cv::Mat R3(3, 3, CV_64F, rArr);
	cv::Mat t3(3, 1, CV_64F, tArr);

	R3.col(0).copyTo(E3.col(0));
	R3.col(1).copyTo(E3.col(1));
	R3.col(2).copyTo(E3.col(2));
	t3.col(0).copyTo(E3.col(3));
	std::cout << "Final Results: " << std::endl;
	std::cout << "A: " << A << std::endl;
	std::cout << "External 1: " << E1 << std::endl;
	std::cout << "External 2: " << E2 << std::endl;
	std::cout << "External 3: " << E3 << std::endl;
	std::cout << "k1 : " << k1 << std::endl;
	std::cout << "k2 : " << k2 << std::endl;

    /* release allocated workspace */
    free (fvec);
    free (fjac);
    free (wa4);
    free (ipvt);
    free (wa1);
    free (wa2);
    free (wa3);
    free (qtf);
    free (diag);
    free (x);

	 return (1);
}


Calibration::Calibration()
{
}


Calibration::~Calibration()
{
}


void Calibration::readInputFile(std::string fileName, std::vector <glm::dvec3>& worldPts, std::vector <glm::dvec2>& imgPts, bool denormalize) {
	std::ifstream file;
	file.open(fileName);

	if (!file.is_open()) {
		throw std::runtime_error("File does not open.");
	}

	std::string line;
	std::string partial;

	std::vector<std::string> tokens;

	worldPts.clear();
	imgPts.clear();
	while (std::getline(file, line)) {     // '\n' is the default delimiter

		std::istringstream iss(line);
		std::string token;
		int i = 0;
		while (std::getline(iss, token, '\t')) {
			// but we can specify a different one
			tokens.push_back(token);
			if (i == 2) {
				worldPts.push_back(glm::dvec3(std::stod(tokens[0].c_str()), std::stod(tokens[1].c_str()), std::stod(tokens[2].c_str())));
				tokens.clear();
			}
			else if (i == 4) {
				if (denormalize) {
    				imgPts.push_back(glm::dvec2((std::stod(tokens[0].c_str()) + 1 ) * 1024.0, (std::stod(tokens[1].c_str()) +1) * 1024.0));
				}
				else {
    				imgPts.push_back(glm::dvec2(std::stod(tokens[0].c_str()), std::stod(tokens[1].c_str()) ));
				}
				tokens.clear();
			}
			++i;
		}
	}
	file.close();
}

void Calibration::calibrate() {
	std::string fn1("corr1.cp");
	std::string fn2("corr2.cp");
	std::string fn3("corr3.cp");
	readInputFile(fn1, worldPts1, imgPts1);
	readInputFile(fn2, worldPts2, imgPts2);
	readInputFile(fn3, worldPts3, imgPts3);
	calcHomography(worldPts1, imgPts1, h1);
	//for (auto itr = imgPts1.begin(); itr != imgPts1.end(); ++itr) {
	//	std::cout << (itr->x/1024.0) - 1 << " ";
	//	std::cout << (itr->y/1024.0) - 1 << " ";
	//	std::cout << std::endl;
	//}
	calcHomography(worldPts2, imgPts2, h2);
	calcRMSE(worldPts1, imgPts1, h1);
	calcRMSE(worldPts2, imgPts2, h2);
	calcIntrinsicMat(h1, h2, A, lambda);
	calcExtrinsicMat(A, h1, E1);
	calcExtrinsicMat(A, h2, E2);
	calcExtrinsicMat(A, h2, E3);
	calcRMSE(worldPts1, imgPts1, A, E1);
	calcRMSE(worldPts2, imgPts2, A, E2);
	calcRMSE(worldPts3, imgPts3, A, E3);
	//optCalib(worldPts1, imgPts1, A, E1, k1, k2);
	optCalib(worldPts1, imgPts1, A, E1, k1, k2);
	//optCalib(worldPts1, imgPts1, A, E2, k1, k2);
	//optCalib(worldPts1, imgPts1, worldPts2, imgPts2, worldPts3, imgPts3, A, E1, E2, E3, k1, k2);
	//optCalib(worldPts3, imgPts3, A, E3, k1, k2);
	calcRMSE(worldPts1, imgPts1, A, E1, k1, k2);
	calcRMSE(worldPts2, imgPts2, A, E2, k1, k2);
	calcRMSE(worldPts3, imgPts3, A, E3, k1, k2);

}

void Calibration::writeFile(std::string fileName, std::vector <glm::dvec3>& worldPts, std::vector <glm::dvec2>& imgPts) {
		std::ofstream file;
		file.open(fileName);
		for (int i = 0; i < worldPts.size(); ++i) {
			file << worldPts[i].x << "\t";
			file << worldPts[i].y << "\t";
			file << worldPts[i].z << "\t";
			file << imgPts[i].x << "\t";
			file << imgPts[i].y << "\n";
		}
}


void Calibration::saveImgPnts(std::vector <glm::dvec3> worldPts, std::vector <glm::dvec2> imgPts, int imgNo) {
	assert(worldPts.size() == imgPts.size());
	if (imgNo == 1) {
		this->worldPts1 = worldPts;
		this->imgPts1 = imgPts;
		std::string fn("corr1.cp");
		writeFile(fn, worldPts, imgPts);
	}
	else if (imgNo == 2) {
		this->worldPts2 = worldPts;
		this->imgPts2 = imgPts;
		std::string fn("corr2.cp");
		writeFile(fn, worldPts, imgPts);
	}
	else if (imgNo == 3) {
		this->worldPts3 = worldPts;
		this->imgPts3 = imgPts;
		std::string fn("corr3.cp");
		writeFile(fn, worldPts, imgPts);
	}
	else {
		throw std::runtime_error("Unknown image number");
	}
}