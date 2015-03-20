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
#include "recon.h"
#include <fstream>
#include <sstream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

using namespace cv;


Recon::Recon()
{
}


Recon::~Recon()
{
}

double Recon::normalized_coord(glm::dvec2 pt) {
	return 0.0;
}

double Recon::denormalized_coord(glm::dvec2 pt) {
	return 0.0;
}

static double computeReprojectionErrors(
	const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}


void Recon::calibrate() {

	vector<vector<Point2f>> img_points;
	vector<vector<Point3f>> world_points;

	cv::Size imageSize;

	for (int i = 0; i < 2; ++i) {
		vector<Point2f> point_buf;
		vector<Point3f> corners;
		string file_name("resources/3dr");
		file_name.append(std::to_string((long long)(i + 1))).append(".jpg"); // .append(" (768x1024).jpg");
		cv::Mat view = imread(file_name, 1);        
		cv::Mat viewGray;

		cv::Size board_size(12, 12);

    	imageSize = view.size();

		bool found;
		found = findChessboardCorners(view, board_size, point_buf); 
		//,
		//	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		cv::cvtColor(view, viewGray, COLOR_BGR2GRAY);
		cornerSubPix(viewGray, point_buf, Size(11, 11),
			Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


		drawChessboardCorners(view, board_size, Mat(point_buf), found);
		imshow(file_name, view);

		for (int i = 0; i < point_buf.size(); ++i) {
			point_buf[i].y = view.rows - point_buf[i].y;
		}

		float squareSize = 38.1; // mm, 1.5 inches

		//for (int i = 0; i < board_size.height; ++i)
		for (int i = board_size.height - 1; i >= 0; --i)
		    //for (int j = board_size.width - 1; j >= 0; --j)
			for (int j = 0; j < board_size.width; ++j)
				corners.push_back(Point3d(float(j*squareSize), float(i*squareSize), 0));


		img_points.push_back(point_buf);
		world_points.push_back(corners);
	}
    // do calibration

	cv::Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<Mat> rvecs; 
	vector<Mat> tvecs;
	vector<float> reprojErrs;
	double totalAvgErr;

	double rms = calibrateCamera(world_points, img_points, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);
	///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
	//totalAvgErr = computeReprojectionErrors(world_points, img_points,
	//	rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
	//printf("RMS error reported by reprojectionError: %g\n", totalAvgErr);

	for (int i = 0; i < world_points.size(); ++i) {
		cv::Mat rotation_matrix;
		cv::Rodrigues(rvecs[i], rotation_matrix);
		cv::Mat E(3, 4, CV_64F);
		rotation_matrix.col(0).copyTo(E.col(0));
		rotation_matrix.col(1).copyTo(E.col(1));
		rotation_matrix.col(2).copyTo(E.col(2));
		tvecs[i].col(0).copyTo(E.col(3));
		cv::Mat proj = cameraMatrix * E;
		projection_matrices_.push_back(proj);
	}

	for (int i = 0; i < world_points.size(); ++i) {
		double totalErr = 0.0;
		double totalPoints = 0;
	    vector<Point2f> imagePoints2;
		for (int j = 0; j < (int)world_points[i].size(); j++)
		{
			cv::Mat world_pnt_mtx(4, 1, CV_64F);
			for (int k = 0; k < 3; ++k) {
				Vec3f vec = world_points[i][j];
				world_pnt_mtx.at<double>(k, 0) = (double) (vec[k]);
			}
			world_pnt_mtx.at<double>(3, 0) = 1.0;
			cv::Mat img_pnt_result = projection_matrices_[i] * world_pnt_mtx;
			cv::Vec3f img_pnt(img_pnt_result);
			double z = img_pnt[2];
			imagePoints2.push_back(Point2f(img_pnt[0]/z, img_pnt[1]/z));
		}
		double err = norm(Mat(img_points[i]), Mat(imagePoints2), CV_L2);
		int n = (int)world_points[i].size();
		totalErr += err*err;
		totalPoints += n;

	    std::cout << std::endl << "RMS error of projection matrix :" << std::sqrt(totalErr / totalPoints);

	}
	std::cout << std::endl;

	test_img_points_ = img_points;
}

void Recon::save_3D_pts(const WPts& pnts) const {
	std::ofstream reconFile;
	reconFile.open("output.txt");
	for (auto x = 0u; x < pnts.size(); ++x) {
		auto& pnt = pnts[x];
		for (int i = 0; i < 3; ++i) {
			reconFile << pnt[i];
			if (i != 2) {
				reconFile << ", ";
			}
		}
		reconFile << std::endl;
	}
	reconFile.close();
}


void Recon::detect_features() {
        
	Mat img_1 = imread("resources/3dr1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat img_2 = imread("resources/3dr2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//Mat img_1 = imread("3dr1 (1024x1024) - Copy.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//Mat img_2 = imread("3dr2 (1024x1024) - Copy.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	if (!img_1.data || !img_2.data)
	{
		std::cout << " --(!) Error reading images " << std::endl;;
	}

	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> keypoints_1, keypoints_2;

	detector.detect(img_1, keypoints_1);
	detector.detect(img_2, keypoints_2);

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_1, descriptors_2;

	extractor.compute(img_1, keypoints_1, descriptors_1);
	extractor.compute(img_2, keypoints_2, descriptors_2);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< DMatch > good_matches;

	for (int i = 0; i < descriptors_1.rows; i++)
	{
		//if (matches[i].distance <= max(2 * min_dist, 0.002))
		if (matches[i].distance <= max(3.0 * min_dist, 0.002))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Show detected matches
	imshow("Good Matches", img_matches);

	img_pts1_.clear();
	img_pts2_.clear();

	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
		//if ((good_matches[i].queryIdx > (keypoints_1.size() - 1)) 
		//    || (good_matches[i].trainIdx > (keypoints_2.size() - 1))) {
		//	printf("Bad point at %d or %d", good_matches[i].queryIdx, good_matches[i].trainIdx);

		//}
		Point2f pt1 = keypoints_1.at(good_matches[i].queryIdx).pt;
		Point2f pt2 = keypoints_2.at(good_matches[i].trainIdx).pt;

		img_pts1_.push_back(glm::dvec2(pt1.x, pt1.y));
		img_pts2_.push_back(glm::dvec2(pt2.x, pt2.y));
		try {
			printf("(%f, %f) -> (%f, %f)\n", keypoints_1.at(good_matches[i].queryIdx).pt.x, keypoints_1.at(good_matches[i].queryIdx).pt.y,
				keypoints_2.at(good_matches[i].trainIdx).pt.x, keypoints_2.at(good_matches[i].trainIdx).pt.y);
		}
		catch (std::exception& ex) {
		    printf("Bad point at %d or %d", good_matches[i].queryIdx, good_matches[i].trainIdx);

		}
	}

	//write_file("recon1.cp", img_pts1_, img_pts2_);

	waitKey(0);
}

void Recon::read_input_file(const std::string file_name, std::vector <glm::dvec2>& img_pts1, std::vector <glm::dvec2>& img_pts2) {
	std::ifstream file;
	file.open(file_name.c_str());
	std::string line;
	cv::Mat img = imread("resources/3dr1.jpg");
	img_pts1.clear();
	img_pts2.clear();
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::string elems;
		std::vector < double > pts;
		while (std::getline(ss, elems, '\t')) {
			pts.push_back(std::stod(elems));
		}
		//img_pts1.push_back(glm::dvec2(pts[1] * 3264.0/1024.0, pts[0] * 2448.0/768.0));
		//img_pts2.push_back(glm::dvec2(pts[3] * 3264.0/1024.0, pts[2] * 2448.0/768.0));
		//img_pts1.push_back(glm::dvec2(pts[0] , pts[1]));
		//img_pts2.push_back(glm::dvec2(pts[2], pts[3]));
		img_pts1.push_back(glm::dvec2(pts[0] , img.rows - pts[1]));
		img_pts2.push_back(glm::dvec2(pts[2], img.rows - pts[3]));
	}
	file.close();
}

void Recon::fill_row(const cv::Mat& P, double coord, cv::Mat& fill_matrix, cv::Mat& B, bool is_y) const {
	int row = is_y;
	for (int j = 0; j < 3; ++j) {
    	fill_matrix.at<double>(0, j) = coord * P.at<double>(2, j) - P.at<double>(row, j);
	}
	B.at<double>(0, 0) = P.at<double>(row, 3) - coord * P.at<double>(2, 3);
}

void Recon::recon_obj(const cv::Mat& A, const cv::Mat& E1, const cv::Mat& E2, const std::vector <glm::dvec2>& img_pts1, const std::vector <glm::dvec2>& img_pts2, WPts& world_pts) {

	assert(img_pts1.size() == img_pts2.size());

    // read file
	cv::Mat proj1 = projection_matrices_[0];
	cv::Mat proj2 = projection_matrices_[1];
    
	std::cout << "Projection Matrix 1: " << std::endl;
	std::cout << proj1 << std::endl;
	std::cout << "Projection Matrix 2: " << std::endl;
	std::cout << proj2 << std::endl;


	for (auto i = 0u; i < img_pts1.size(); ++i) {
	    cv::Mat D(4, 3, CV_64F);
		cv::Mat b(4, 1, CV_64F);

        // fill rows from point 1
		for (auto x = 0u; x < 2; ++x) {
			cv::Mat row_D(1, 3, CV_64F);
			cv::Mat row_B(1, 1, CV_64F);
			fill_row(proj1, img_pts1[i][x], row_D, row_B, x);
			row_D.row(0).copyTo(D.row(x));
			row_B.row(0).copyTo(b.row(x));
		}
		for (auto x = 0u; x < 2; ++x) {
			cv::Mat row_D(1, 3, CV_64F);
			cv::Mat row_B(1, 1, CV_64F);
			fill_row(proj2, img_pts2[i][x], row_D, row_B, x);
			row_D.row(0).copyTo(D.row(x+2));
			row_B.row(0).copyTo(b.row(x+2));
		}
		cv::Mat XYZ(3, 1, CV_64F);
		cv::solve(D, b, XYZ, DECOMP_SVD);
		//cv::Mat D_inv = D.inv(DECOMP_SVD);
		//cv::Mat XYZ = D_inv * b;

		cv::Mat b_test = D * XYZ;
		cv::Mat results = b - b_test;
		//std::cout << "error: " << results << std::endl;
		//std::cout << "XYZ : " << XYZ << std::endl;
		//std::cout << "D Matrix : " << D << std::endl;
		//std::cout << "b Matrix : " << b << std::endl;
		//std::cout << "Img Points 1 : " << img_pts1[i].x << ", " << img_pts1[i].y << std::endl;
		//std::cout << "Img Points 2 : " << img_pts2[i].x << ", " << img_pts2[i].y << std::endl;
		world_pts.push_back(glm::dvec3(XYZ.at<double>(0, 0), XYZ.at<double>(1, 0), XYZ.at<double>(2, 0)));
	}

	std::cout << "X = [";
	for (int i = 0; i < world_pts.size(); ++i) {
		std::cout << world_pts[i].x << ";";
	}
	std::cout << "], ";

	std::cout << "Y = [";
	for (int i = 0; i < world_pts.size(); ++i) {
		std::cout << world_pts[i].y << ";";
	}
	std::cout << "], ";

	std::cout << "Z = [";
	for (int i = 0; i < world_pts.size(); ++i) {
		std::cout << world_pts[i].z << ";";
	}
	std::cout << "]";
}

glm::dvec3 project_point(const glm::dvec3& world_pt, const glm::dvec3& normal, const glm::dvec3& pt_on_plane) {
	return (glm::dot(world_pt, normal) - pt_on_plane);
}

cv::Mat project_point_mat(const cv::Mat world_pt, const cv::Mat normal, const cv::Mat pt_on_plane) {
	return (world_pt.dot(normal) - pt_on_plane);
}

bool cmpx(const cv::Point2f& left, const cv::Point2f& right) {
	return (left.x < right.x);
}

bool cmpy(const cv::Point2f& left, const cv::Point2f& right) {
	return (left.y < right.y);
}


void Recon::texturize_triangle(const WPts& triangle, const std::vector<cv::Point2f>& triangle_img_coords, std::vector<glm::dvec2>& texture_coords, GLuint& tex) {
	assert(triangle.size() == 3);
	assert(triangle.size() == triangle_img_coords.size());

	cv::Vec3f triangles_vec3f[3];
	for (int i = 0; i < 3; ++i) {
		triangles_vec3f[i] = cv::Vec3f(triangle[i][0], triangle[i][1], triangle[i][2]);
	}
	cv::Vec3f X = triangles_vec3f[1] - triangles_vec3f[0];
	cv::Vec3f Y = triangles_vec3f[2] - triangles_vec3f[0];
	cv::Vec3f Z = X.cross(Y);

	Y = Z.cross(X);
	//cv::normalize(X, X, 0.0, 1.0, NORM_MINMAX);
	//cv::normalize(Y, Y, 0.0, 1.0, NORM_MINMAX);
	//cv::normalize(Z, Z, 0.0, 1.0, NORM_MINMAX);

	cv::Mat A(3, 3, CV_32F);
	cv::Mat(X).col(0).copyTo(A.col(0));
	cv::Mat(Y).col(0).copyTo(A.col(1));
	cv::Mat(Z).col(0).copyTo(A.col(2));

	//std::cout << "A: " << A << std::endl;

	cv::Mat invA = A.inv();
	//std::cout << "inv A: " << invA << std::endl;

	cv::Mat image = cv::imread("resources/3dr1.jpg");
	std::vector<cv::Point2f> triangle_2d;
	//std::cout << "transformed triangles";
	for (int i = 0; i < 3; ++i) {
		cv::Mat transformed_triangle = (cv::Mat(invA * cv::Mat(triangles_vec3f[i] - triangles_vec3f[0]))).col(0);
		//cv::abs(transformed_triangle, transformed_triangle, 0.0, 1.0, NORM_MINMAX);
		cv::abs(transformed_triangle);
		float x = transformed_triangle.at<float>(0, 0);
		float y = transformed_triangle.at<float>(1, 0);
		//std::cout << x << ", " << y << std::endl;
		triangle_2d.push_back(cv::Point2f(x, y));
		//texture_coords.push_back(glm::dvec2(x, y));

        //backup
		texture_coords.push_back(glm::dvec2((float)(triangle_img_coords[i].x) / (float)(image.cols),
			(float)(triangle_img_coords[i].y) / (float)(image.rows)));
	}
		//texture_coords.push_back(glm::dvec2(1.f, 1.f));
		//texture_coords.push_back(glm::dvec2(0.f, 0.f));
		//texture_coords.push_back(glm::dvec2(1.f, 0.f));

	float maxX = (*std::max_element(triangle_2d.begin(), triangle_2d.end(), [](const cv::Point2f& left, const cv::Point2f& right) {
		return (left.x < right.x);
	})).x;

	float maxY = (*std::max_element(triangle_2d.begin(), triangle_2d.end(), [](const cv::Point2f& left, const cv::Point2f& right) {
		return (left.y < right.y);
	})).y;

	cv::Mat transform = cv::getAffineTransform(triangle_img_coords, triangle_2d);


	cv::Mat warped_image = Mat::zeros(image.rows, image.cols, image.type());

	cv::warpAffine(image, warped_image, transform, warped_image.size());

	//imshow("warped image", warped_image);
	//waitKey(0);

	cv::Rect rect(0, 0, maxX, maxY);
	cv::Mat clipped_image = warped_image(rect);

	//imshow("warped image", clipped_image);
	//waitKey(0);
	//cv::Mat flipped;
	//cv::flip(image, flipped, 0);
	//image = flipped;

	cv::flip(clipped_image, clipped_image, 0);
	//glGenTextures(1, &tex);
	//glBindTexture(GL_TEXTURE_2D, tex);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//// Set texture clamping method
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


	//glTexImage2D(GL_TEXTURE_2D,     // Type of texture
	//	0,
	//	GL_RGB,
	//	clipped_image.cols,
	//	clipped_image.rows,
	//	0,
	//	GL_BGR,
	//	GL_UNSIGNED_BYTE,
	//	clipped_image.ptr());


	glBindTexture(GL_TEXTURE_2D, 0);
}


void Recon::traingulate_pts(const WPts& world_pnts, WPts& triangles, std::vector<GLuint>& texture_ids, std::vector<glm::dvec2>& texture_coords, const cv::Mat& A, const cv::Mat& E) {

	const cv::Mat proj = projection_matrices_[0];
	//double origin[] = { 0.0, 0.0, 0.0 };
	//cv::Mat some_pnt(3, 1, CV_32F, origin);

	//double z1[] = { 0.0, 0.0, 1.0 };
	//cv::Mat pt_on_plane(3, 1, CV_32F, z1);
	//cv::Mat normal = some_pnt - pt_on_plane;

	std::vector<cv::Point2f> projected_pts;
	for (int i = 0u; i < world_pnts.size(); ++i) {
		// project point
		cv::Mat world_pnt(4, 1, CV_64F);
		for (int x = 0; x < 3; ++x) {
			world_pnt.at<double>(x, 0) = (double)world_pnts[i][x];
		}
		world_pnt.at<double>(3, 0) = 1.0;
		cv::Mat projected_pt = proj * world_pnt;
		double z = projected_pt.at<double>(2, 0);
		projected_pts.push_back(cv::Point2f(projected_pt.at<double>(0, 0)/z, projected_pt.at<double>(1, 0)/z));
	}

	assert(world_pnts.size() == projected_pts.size());

    // construct rect
	float maxX = (*std::max_element(projected_pts.begin(), projected_pts.end(), cmpx)).x;
	float minX = (*std::min_element(projected_pts.begin(), projected_pts.end(), cmpx)).x;
	float maxY = (*std::max_element(projected_pts.begin(), projected_pts.end(), cmpy)).y;
	float minY = (*std::min_element(projected_pts.begin(), projected_pts.end(), cmpy)).y;

	cv::Rect rect(minX - 1, minY - 1, maxX + minX, maxY + minY);

	cv::Subdiv2D subdiv(rect);
	std::cout << std::endl;
	for (int i = 0u; i < projected_pts.size(); ++i) {
		std::cout << projected_pts[i].x << ", " << projected_pts[i].y << std::endl;
		subdiv.insert(projected_pts[i]);
	}

	std::vector<cv::Vec6f> triangle_list;
	subdiv.getTriangleList(triangle_list);

	const float t = 1e-6;
	for (auto i = 0u; i < triangle_list.size(); ++i) {
		auto vec6 = triangle_list[i];
		Point2f triangle_pts[3];
		std::vector<int> triangle_index;
		for (int x = 0; x < 3; ++x) {
			for (int k = 0; k < projected_pts.size(); ++k) {
				if (projected_pts[k].x + t > vec6[2*x]
					&& projected_pts[k].x - t < vec6[2*x]
                    && projected_pts[k].y + t > vec6[(2*x) + 1]
					&& projected_pts[k].y - t < vec6[(2*x) + 1]) {
					triangle_index.push_back(k);
					break;
				}
			}
		}

		if (triangle_index.size() == 3) {
			WPts triangle;
	        std::vector<cv::Point2f> image_coords;
			for (auto k = 0u; k < triangle_index.size(); ++k) {
        		triangles.push_back(world_pnts[triangle_index[k]]);
				triangle.push_back(world_pnts[triangle_index[k]]);
				image_coords.push_back(projected_pts[triangle_index[k]]);
			}
			GLuint tex = 0;
			texturize_triangle(triangle, image_coords, texture_coords, tex);
			texture_ids.push_back(tex);
		}
	}

	//assert(3 * triangle_list.size() == triangles.size());

	//cv::Mat image = cv::imread("resources/3dr1 (768x1024).jpg");
	//cv::Mat flipped = image.clone();
	//cv::flip(flipped, image, 0);

	glGenVertexArrays(1, &recon_vao);
	glGenBuffers(3, recon_vbo);

	//glGenTextures(1, &backup_tex);
	//glBindTexture(GL_TEXTURE_2D, backup_tex);
	////use fast 4-byte alignment (default anyway) if possible
	//glPixelStorei(GL_UNPACK_ALIGNMENT, (image.step & 3) ? 1 : 4);

	////set length of one complete row in data (doesn't need to equal image.cols)
	//glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image.elemSize());

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//// Set texture clamping method
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


	//glTexImage2D(GL_TEXTURE_2D,     // Type of texture
	//	0,
	//	GL_RGB,
	//	image.cols,
	//	image.rows,
	//	0,
	//	GL_BGR,
	//	GL_UNSIGNED_BYTE,
	//	image.ptr());


	//glBindTexture(GL_TEXTURE_2D, 0);

    //tdogl::Bitmap bmp3 = tdogl::Bitmap::bitmapFromFile("resources/3dr1 (1024x1024).jpg");
    tdogl::Bitmap bmp3 = tdogl::Bitmap::bitmapFromFile("resources/3dr1.jpg");
    bmp3.flipVertically();
    gTex = new tdogl::Texture(bmp3);

}

float rotate_angle = 0.f;

void Recon::draw() {
	glBindVertexArray(recon_vao);

	std::vector<glm::vec3> connecting_line_vertices;
	std::vector<glm::vec3> connecting_line_vertice_colors;
	std::vector<glm::vec2> connecting_line_texture_coords;

	glm::vec4 init_vtx_pos(0.f, 0.f, 0.f, 1.f);
	glm::vec3 distance(0.f);
	for (auto i = 0u; i < world_pts_triangles_.size(); ++i) {
		distance += glm::vec3(0.f) - glm::vec3(world_pts_triangles_[i]);
	}
	glm::vec3 avg_distance = distance / (float)(world_pts_triangles_.size());
    glm::mat4 translated_matrix = glm::translate(glm::mat4(1.f), avg_distance);
	for (auto i = 0u; i < world_pts_triangles_.size(); ++i) {

		glm::vec4 vtx1 = translated_matrix * glm::vec4(world_pts_triangles_[i], 1.0);
		//glm::vec4 vtx1 = glm::vec4(world_pts_triangles_[i], 1.0);
        
		connecting_line_vertices.push_back(glm::vec3(vtx1.x, vtx1.y, vtx1.z));
		glm::vec3 vtx_color((float)(std::rand() % 255) / 255.f, (float)(std::rand() % 255) / 255.f, (float)(std::rand() % 255) / 255.f);
		//glm::vec3 vtx_color(1.f);
		connecting_line_vertice_colors.push_back(vtx_color);
		connecting_line_texture_coords.push_back(glm::vec2(texture_coords_[i].x, texture_coords_[i].y));
	}

	glBindBuffer(GL_ARRAY_BUFFER, recon_vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, connecting_line_vertices.size() * sizeof(glm::vec3), &connecting_line_vertices[0], GL_STREAM_DRAW);

	// connect the xyz to the "vert" attribute of the vertex shader
	glEnableVertexAttribArray(gProgram->attrib("vert"));
	glVertexAttribPointer(gProgram->attrib("vert"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	//glBindBuffer(GL_ARRAY_BUFFER, recon_vbo[1]);
	//glBufferData(GL_ARRAY_BUFFER, connecting_line_vertice_colors.size() * sizeof(glm::vec3), &connecting_line_vertice_colors[0], GL_STREAM_DRAW);

	//glEnableVertexAttribArray(gProgram->attrib("vertColor"));
	//glVertexAttribPointer(gProgram->attrib("vertColor"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, recon_vbo[2]);
	glBufferData(GL_ARRAY_BUFFER, connecting_line_texture_coords.size() * sizeof(glm::vec2), &connecting_line_texture_coords[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(gProgram->attrib("vertTexCoord"));
	glVertexAttribPointer(gProgram->attrib("vertTexCoord"), 2, GL_FLOAT, GL_FALSE, 0, 0);

	gProgram->setUniform("model", glm::scale(glm::rotate(glm::mat4(1.f), rotate_angle++, glm::vec3(0.f, 1.f, 0.f)), glm::vec3(-0.01f, 0.01f, 0.01f)));

		glBindTexture(GL_TEXTURE_2D, gTex->object());
		gProgram->setUniform("tex", 0); //set to 0 because the texture is bound to GL_TEXTURE0
		glDrawArrays(GL_TRIANGLES, 0, connecting_line_vertices.size());
	//for (auto i = 0u; i < connecting_line_vertices.size(); i+=3) {
	//	//glBindTexture(GL_TEXTURE_2D, texture_ids_[i % 3]);

	//    glDrawArrays(GL_TRIANGLES, i, 3);
	//}
    

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindVertexArray(0);

}

void Recon::reconstruct_obj(const int corr_no, Calibration* calib) {
	if (corr_no == 1) {
		read_input_file("recon1.cp", img_pts1_, img_pts2_);
		recon_obj(calib->getA(), calib->getE1(), calib->getE2(), img_pts1_, img_pts2_, world_pts_);
		traingulate_pts(world_pts_, world_pts_triangles_, texture_ids_, texture_coords_, calib->getA(), calib->getE1());
		//save_3D_pts(world_pts_);
	}

}

void Recon::write_file(const std::string& file_name, const std::vector <glm::dvec2>& img_pts1, const std::vector <glm::dvec2>& img_pts2) {
		std::ofstream file;
		file.open(file_name);
		for (int i = 0; i < img_pts1.size(); ++i) {
			file << img_pts1[i].x << "\t";
			file << img_pts1[i].y << "\t";
			file << img_pts2[i].x << "\t";
			file << img_pts2[i].y << "\n";
		}
		file.close();
}


void Recon::save_corr_pnts(const std::vector <glm::dvec2>& img_pts1, const std::vector <glm::dvec2> img_pts2, const int corr_no) {
	assert(img_pts1.size() == img_pts2.size());
	if (corr_no == 1) {
		this->img_pts1_ = img_pts1;
		this->img_pts2_ = img_pts2;
		std::string fn("recon1.cp");
		write_file(fn, img_pts1, img_pts2);
	}
	else if (corr_no == 2) {
		this->img_pts3_ = img_pts1;
		this->img_pts4_ = img_pts2;
		std::string fn("recon2.cp");
		write_file(fn, img_pts1, img_pts2);
	}
	else {
		throw std::runtime_error("Unknown image number");
	}
}
