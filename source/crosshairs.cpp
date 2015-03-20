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
#include "crosshairs.h"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <fstream>
#include <sstream>

using namespace cv;


void CrossHairs::draw(glm::dvec3 center) {
	int i = 0;
	glBindVertexArray(gVAO);
	//for (auto& itr : pos) {
	for (auto itr = pos.begin(); itr != pos.end(); ++itr) {
		gProgram->setUniform("model", glm::translate(glm::mat4(1.0f), glm::vec3(itr->second)));
        
    // bind the texture and set the "tex" uniform in the fragment shader
    //glActiveTexture(GL_TEXTURE0);
    //glBindTexture(GL_TEXTURE_2D, gTexture->object());
    //gProgram->setUniform("tex", 0); //set to 0 because the texture is bound to GL_TEXTURE0

		// bind the VAO (the triangle)

		// draw the VAO
		glDrawArrays(GL_LINES, 4*i, 4*i+4);

		// unbind the VAO, the program and the texture
		++i;
	}
	glBindVertexArray(0);
}

#define PI 3.14159265359

void CrossHairs::detect_corners(int imgNo, double w, double h) {
	auto result = corners_.find(imgNo);
	if ( result != corners_.end()) {
		auto corner_positions = result->second;
		if (pos.size() > 0) {
			for (auto itr = pos.begin(); itr != pos.end(); ++itr) {
				glm::dvec3 pos = itr->second;
		        double min = 1e+25;
				int min_index = 0;
				for (int i = 0; i < corner_positions.size(); ++i) {
					auto corner_pos = corner_positions[i];
					double chDist = std::pow(corner_pos.x - pos.x, 2) + std::pow(corner_pos.y - pos.y, 2);
					if (chDist < min) {
						min = chDist;
						min_index = i;
					}
				}
				std::cout << min_index <<", " << min << ", -- " << pos.x << "," << pos.y << " -> ";
			    itr->second = glm::dvec3(corner_positions[min_index].x, corner_positions[min_index].y, itr->second.z);
				std::cout << itr->second.x << "," << itr->second.y << std::endl;
			}
		}
	}
	else {
		char* corners_window = "Corners detected";
		Mat src, src_gray;
		std::string fileName("resources/3dr");
		fileName.append(std::to_string(static_cast<long long>(imgNo))).append(" (1024x1024).jpg");

		src = imread(fileName, 1);
		cvtColor(src, src_gray, CV_BGR2GRAY);

		int thresh = 150;
		int max_thresh = 255;

		Mat dst, dst_norm, dst_norm_scaled;
		dst = Mat::zeros(src.size(), CV_32FC1);

		/// Detector parameters
		int blockSize = 2;
		int apertureSize = 3;
		double k = 0.04;

		/// Detecting corners
		cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);

		/// Normalizing
		normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
		convertScaleAbs(dst_norm, dst_norm_scaled);

		/// Drawing a circle around corners
		std::vector < glm::vec3 > corners;
		for (int j = 0; j < dst_norm.rows; j++) {
			for (int i = 0; i < dst_norm.cols; i++) {
				if ((int)dst_norm.at<float>(j, i) > thresh) {
					//circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
					//double cX = i - (w / 2.0);
					//double iW = tan(22.5f * (double)PI / 180) * 0.1 * w / h;
					//double oW = (3.5f / 0.1) * iW;
					//double mX = oW * cX / (w / 2.0);

					//double cY = j - (h / 2.0);
					//double oY = oW * h / w;
					//double mY = oY * cY / (h / 2.0);

					double mX = ((float)i/512.f) - 1.f;
					double mY = ((float)j/512.f) - 1.f;

					corners.push_back(glm::vec3(mX, -mY, 0.f));
					//std::cout << mX << ", " << mY << std::endl;
				}
			}
		}
		corners_[imgNo] = corners;
		/// Showing the result
		namedWindow(corners_window, CV_WINDOW_AUTOSIZE);
		imshow(corners_window, dst_norm_scaled);

	}

}

void CorrCrossHairs::read_input_file(const std::string file_name) {
	std::ifstream file;
	file.open(file_name.c_str());
	std::string line;
	cv::Mat img = imread("resources/3dr1.jpg");
	int i = 0;
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
		glm::dvec2 pos1 = glm::dvec2(pts[0] , img.rows - pts[1]);
		glm::dvec2 pos2 = (glm::dvec2(pts[2], img.rows - pts[3]));
		float half_width = (float)(img.cols) / 2.f;
		float half_height = (float)(img.rows)/ 2.f;
		pos1.x = (pos1.x - half_width) / half_width;
		pos1.y = (pos1.y - half_height) / half_height;
		pos2.x = (pos2.x - half_width) / half_width;
		pos2.y = (pos2.y - half_height) / half_height;
		adjust(2 * i, pos1.x, pos1.y);
		adjust((2 * i) + 1, pos2.x, pos2.y);

		++i;
	}
	file.close();
}

void CorrCrossHairs::draw(glm::dvec3 center, glm::mat4 model1, glm::mat4 model2) {
	int i = 0;
	glBindVertexArray(gVAO);
	//for (auto& itr : pos) {
	for (auto itr = pos.begin(); itr != pos.end(); ++itr) {
		if (i % 2 == 0) {
			gProgram->setUniform("model", model1 * glm::translate(glm::mat4(1.0f), glm::vec3(itr->second)));
		}
		else {
			gProgram->setUniform("model", model2 * glm::translate(glm::mat4(1.0f), glm::vec3(itr->second)));
		}
        
    // bind the texture and set the "tex" uniform in the fragment shader
    //glActiveTexture(GL_TEXTURE0);
    //glBindTexture(GL_TEXTURE_2D, gTexture->object());
    //gProgram->setUniform("tex", 0); //set to 0 because the texture is bound to GL_TEXTURE0

		// bind the VAO (the triangle)

		// draw the VAO
		glDrawArrays(GL_LINES, 4*i, 4*i+4);

		// unbind the VAO, the program and the texture
		++i;
	}


	glBindVertexArray(connecting_line_VAO);

	std::vector<glm::vec3> connecting_line_vertices;
	std::vector<glm::vec3> connecting_line_vertice_colors;

	glm::vec4 init_vtx_pos(0.f, 0.f, 0.f, 1.f);
	for (auto i = 0u; i < pos.size(); ++i) {
		glm::mat4 translated_matrix1;
		if (i % 2 == 0) {
			translated_matrix1 = model1 * glm::translate(glm::mat4(1.f), glm::vec3(pos[i]));
		}
		else {
			translated_matrix1 = model2 * glm::translate(glm::mat4(1.f), glm::vec3(pos[i]));
		}

		glm::vec4 vtx1 = translated_matrix1 * init_vtx_pos;
        
		connecting_line_vertices.push_back(glm::vec3(vtx1.x, vtx1.y, vtx1.z));
		glm::vec3 vtx_color = (i % 2) ? color: color2;
		connecting_line_vertice_colors.push_back(vtx_color);
	}

	glBindBuffer(GL_ARRAY_BUFFER, connecting_line_VBO[0]);
	glBufferData(GL_ARRAY_BUFFER, connecting_line_vertices.size() * sizeof(glm::vec3), &connecting_line_vertices[0], GL_STREAM_DRAW);

	// connect the xyz to the "vert" attribute of the vertex shader
	glEnableVertexAttribArray(gProgram->attrib("vert"));
	glVertexAttribPointer(gProgram->attrib("vert"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, connecting_line_VBO[1]);
	glBufferData(GL_ARRAY_BUFFER, connecting_line_vertice_colors.size() * sizeof(glm::vec3), &connecting_line_vertice_colors[0], GL_STREAM_DRAW);

	glEnableVertexAttribArray(gProgram->attrib("vertColor"));
	glVertexAttribPointer(gProgram->attrib("vertColor"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	gProgram->setUniform("model", glm::mat4(1.f));
	glDrawArrays(GL_LINES, 0, connecting_line_vertices.size());
    

	glBindVertexArray(0);
}

void CrossHairs::adjust(double mX, double mY) {
	//float mDist = std::pow(mX, 2) + std::pow(mY, 2);
	float min = 1e+25;
	if (pos.size() > 0) {
		auto minItr = pos.begin();
		for (auto itr = pos.begin(); itr != pos.end(); ++itr) {
			glm::dvec3 pos = itr->second;
			float chDist = std::pow(mX - pos.x, 2) + std::pow(mY - pos.y, 2);
			if (chDist < min) {
				min = chDist;
				minItr = itr;
			}
		}
		minItr->second = glm::dvec3(mX, mY, minItr->second.z);
	}
}

void CrossHairs::adjust(int crosshair, double mX, double mY) {
	pos[crosshair] = glm::dvec3(mX, mY, pos[crosshair].z);
}


void CrossHairs::init(Program* gProgram) {

	this->gProgram = gProgram;
	// make and bind the VAO
	glGenVertexArrays(1, &gVAO);
	glBindVertexArray(gVAO);

	// make and bind the VBO
	glGenBuffers(1, &gVBO);
	glBindBuffer(GL_ARRAY_BUFFER, gVBO);

	std::vector<glm::vec3> vtcs;
	std::vector<glm::vec3> vtcColors;

	float cw = .1f;
	float ch = .1f;
	float iPntX = 0.f;
	float iPntY = 0.f;
	float z = 0.f;

	for (int i = 0; i < pos.size(); ++i) {
		// cross hair horizontal line
		vtcs.push_back(glm::vec3(iPntX - (cw / 2.f), iPntY, z));
		vtcs.push_back(glm::vec3(iPntX + (cw / 2.f), iPntY, z));

		// cross hair vertical line
		vtcs.push_back(glm::vec3(iPntX, iPntY - (ch / 2.f), z));
		vtcs.push_back(glm::vec3(iPntX, iPntY + (ch / 2.f), z));

		for (int j = 0; j < 4; ++j) {
			vtcColors.push_back(color);
		}
	}


	glBufferData(GL_ARRAY_BUFFER, vtcs.size() * sizeof(glm::vec3), &vtcs[0], GL_STATIC_DRAW);

	// connect the xyz to the "vert" attribute of the vertex shader
	glEnableVertexAttribArray(gProgram->attrib("vert"));
	glVertexAttribPointer(gProgram->attrib("vert"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	// make and bind the VBO
	glGenBuffers(1, &gVBO2);
	glBindBuffer(GL_ARRAY_BUFFER, gVBO2);

	glBufferData(GL_ARRAY_BUFFER, vtcColors.size() * sizeof(glm::vec3), &vtcColors[0], GL_STATIC_DRAW);

	// connect the xyz to the "vert" attribute of the vertex shader
	glEnableVertexAttribArray(gProgram->attrib("vertColor"));
	glVertexAttribPointer(gProgram->attrib("vertColor"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	// connect the uv coords to the "vertTexCoord" attribute of the vertex shader
	//glEnableVertexAttribArray(gProgram->attrib("vertTexCoord"));
	//glVertexAttribPointer(gProgram->attrib("vertTexCoord"), 2, GL_FLOAT, GL_TRUE,  5*sizeof(GLfloat), (const GLvoid*)(3 * sizeof(GLfloat)));


	// unbind the VAO
	glBindVertexArray(0);

}

CrossHairs::CrossHairs(int totInRow, double w, double h, glm::vec3 color) : totInRow(totInRow), w(w), h(h), color(color) {
	double wOffset = w / (double) (totInRow + 1);
	double hOffset = h / (double) (totInRow + 1);
	for (int i = 0; i < totInRow; ++i) {
		for (int j = 0; j < totInRow; ++j) {
			double x = wOffset * (i + 1);
			double y = hOffset * (j + 1);
			double z = 0.0;
			double normX = (x - w / 2.0) * 2.0 / w;
			double normY = (y - h / 2.0)* 2.0 / h;
			pos.insert(std::make_pair(j + totInRow * i, glm::dvec3(normX, normY, z)));
			//std::cout << normX << ", " << normY << std::endl;
		}
	}
};


CrossHairs::~CrossHairs()
{
	gProgram = nullptr;
}

CorrCrossHairs::CorrCrossHairs(int totInRow, double w, double h, glm::vec3 color1, glm::vec3 color2) : CrossHairs((totInRow), (w), (h), (color1)), color2(color2) {
	double wOffset = w / (double) (totInRow + 1);
	double hOffset = h / (double) (totInRow + 1);
	pos.clear();
	for (int i = 0; i < totInRow; ++i) {
		for (int j = 0; j < totInRow; ++j) {
			double x = wOffset * (i + 1);
			double y = hOffset * (j + 1);
			double z = 0.0;
			double normX = (x - w / 2.0) * 2.0 / w;
			double normY = (y - h / 2.0)* 2.0 / h;
			double offset = 0.5;
			pos.insert(std::make_pair(((j * 2)+ (totInRow * i * 2)), glm::dvec3(normX - offset, normY, z)));
			pos.insert(std::make_pair(((j * 2) + (totInRow * i * 2) + 1), glm::dvec3(normX + offset, normY, z)));
			//std::cout << normX << ", " << normY << std::endl;
		}
	}
};

void CorrCrossHairs::init(Program* gProgram, glm::mat4 projInv, glm::mat4 camInv) {

	proj_inv_ = projInv;
	cam_inv_ = camInv;
	this->gProgram = gProgram;
	// make and bind the VAO
	glGenVertexArrays(1, &gVAO);
	glBindVertexArray(gVAO);

	// make and bind the VBO
	glGenBuffers(1, &gVBO);
	glBindBuffer(GL_ARRAY_BUFFER, gVBO);

	std::vector<glm::vec3> vtcs;
	std::vector<glm::vec3> vtcColors;

	float cw = .1f;
	float ch = .1f;
	float iPntX = 0.f;
	float iPntY = 0.f;
	float z = 0.f;

	for (int i = 0; i < (pos.size() * 2); ++i) {
		// cross hair horizontal line
		vtcs.push_back(glm::vec3(iPntX - (cw / 2.f), iPntY, z));
		vtcs.push_back(glm::vec3(iPntX + (cw / 2.f), iPntY, z));

		// cross hair vertical line
		vtcs.push_back(glm::vec3(iPntX, iPntY - (ch / 2.f), z));
		vtcs.push_back(glm::vec3(iPntX, iPntY + (ch / 2.f), z));

		if (i % 2) {
			for (int j = 0; j < 4; ++j) {
				vtcColors.push_back(color);
			}
		}
		else {
			for (int j = 0; j < 4; ++j) {
				vtcColors.push_back(color2);
			}
		}
	}



	glBufferData(GL_ARRAY_BUFFER, vtcs.size() * sizeof(glm::vec3), &vtcs[0], GL_STATIC_DRAW);

	// connect the xyz to the "vert" attribute of the vertex shader
	glEnableVertexAttribArray(gProgram->attrib("vert"));
	glVertexAttribPointer(gProgram->attrib("vert"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	// make and bind the VBO
	glGenBuffers(1, &gVBO2);
	glBindBuffer(GL_ARRAY_BUFFER, gVBO2);

	glBufferData(GL_ARRAY_BUFFER, vtcColors.size() * sizeof(glm::vec3), &vtcColors[0], GL_STATIC_DRAW);

	// connect the xyz to the "vert" attribute of the vertex shader
	glEnableVertexAttribArray(gProgram->attrib("vertColor"));
	glVertexAttribPointer(gProgram->attrib("vertColor"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	// connect the uv coords to the "vertTexCoord" attribute of the vertex shader
	//glEnableVertexAttribArray(gProgram->attrib("vertTexCoord"));
	//glVertexAttribPointer(gProgram->attrib("vertTexCoord"), 2, GL_FLOAT, GL_TRUE,  5*sizeof(GLfloat), (const GLvoid*)(3 * sizeof(GLfloat)));

	glGenVertexArrays(1, &connecting_line_VAO);
	glBindVertexArray(connecting_line_VAO);

	glGenBuffers(2, connecting_line_VBO);

	// unbind the VAO
	glBindVertexArray(0);

	read_input_file("recon1.cp");

}

void CorrCrossHairs::addPoint() {
    
}