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
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <opencv2/core/core.hpp>
#include "calibration.h"
#include "tdogl/Texture.h"
#include <unordered_map>

typedef std::vector<glm::dvec2> IPts;
typedef std::vector<glm::dvec3> WPts;
typedef std::vector<cv::Vec6f> TriangleList;
typedef std::pair<int, std::pair<cv::Vec3i, cv::Vec3i> > UniqueColorPair;

class Recon
{
private:
	cv::Rect validRoi_[2];
	cv::Mat rmap_[2][2];

	float yoffset_;
	GLuint recon_vao;
	GLuint recon_vbo[3];
	Program* gProgram;
	std::vector<glm::dvec2> img_pts1_, img_pts2_, img_pts3_, img_pts4_;
	std::vector<glm::dvec3> world_pts_;
	std::vector<glm::dvec3> world_pts_triangles_;
	std::vector<cv::Vec6f> triangleList;
	double normalized_coord(glm::dvec2 pt);
	double denormalized_coord(glm::dvec2 pt);
	void fill_row(const cv::Mat& P, double coord, cv::Mat& fill_matrix, cv::Mat& B, const bool is_y) const;
	void recon_obj(const cv::Mat& A, const cv::Mat& E1, const cv::Mat& E2, const std::vector <glm::dvec2>& img_pts1, const std::vector <glm::dvec2>& img_pts2, std::vector<glm::dvec3>& world_pts);
	void write_file(const std::string& file_name, const std::vector <glm::dvec2>& img_pts1, const std::vector <glm::dvec2>& img_pts2);
	void read_input_file(const std::string file_name, std::vector <glm::dvec2>& img_pts1, std::vector <glm::dvec2>& img_pts2);
	void save_3D_pts(const WPts& pnts) const;
	void traingulate_pts(const WPts& pnts, WPts& triangleList, std::vector<GLuint>& texture_ids, std::vector<glm::dvec2>& texture_coords, const cv::Mat& A, const cv::Mat& E);
	void texturize_triangle(const WPts& triangle, const std::vector<cv::Point2f>& img_coords, std::vector<glm::dvec2>& texture_coords, GLuint& tex, cv::Mat& image);
	std::vector<cv::Mat> projection_matrices_;
	std::vector<cv::Mat> rectification_matrices_;
	std::vector<std::vector<cv::Point2f>> test_img_points_; 
	std::vector<GLuint> texture_ids_; 
	std::vector<glm::dvec2> texture_coords_;
	GLuint backup_tex;
	tdogl::Texture* gTex;
	void rectify_images(std::vector<cv::Mat>& imgs_left);
	void compute_correlation_per_image(std::unordered_map<int, std::unordered_map< int, std::vector<UniqueColorPair> > >& unique_colors_left,
		std::unordered_map<int, std::unordered_map< int, std::vector<UniqueColorPair> > >& unique_colors_right,
		std::vector <glm::dvec2>& img_pts1, std::vector <glm::dvec2>& img_pts2);
	void init_imgs(std::vector<cv::Mat>& calibImgs, std::vector<std::string> img_filenames, std::string albedo_filename, bool is_left);

	void get_unique_edges(const std::vector<cv::Mat>& calibImgs,
		std::unordered_map<int, std::unordered_map< int, std::vector<UniqueColorPair> > >& unique_colors_per_image,
		bool is_right);
	cv::Size image_size_;
    
public:
	void compute_correlation(std::vector <glm::dvec2>& img_pts1, std::vector <glm::dvec2>& img_pts2);
	inline void set_gprogram(Program* g_program) { gProgram = g_program; };
	void calibrate();
	void save_corr_pnts(const std::vector <glm::dvec2>& img_pts1, const std::vector <glm::dvec2> img_pts2, const int corr_no);
	void detect_features();
	void reconstruct_obj(const int corr_no, Calibration* calib);
	void draw();
	Recon();
	~Recon();
};

