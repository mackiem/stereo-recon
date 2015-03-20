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

#include "glm/glm.hpp";
#include <vector>;
#include <map>;
#include "tdogl/Program.h"

using namespace tdogl;

class CrossHairs
{
protected:
	double w;
	double h;
	GLuint gVAO;
	GLuint gVBO;
	GLuint gVBO2;
	Program* gProgram;
	glm::vec3 color;
	std::map<int, std::vector<glm::vec3>> corners_;

public:
	int totInRow;
	std::map<int, glm::dvec3> pos;
	CrossHairs(int totInRow, double w, double h, glm::vec3 color);
	virtual void init(Program* gProgram);
	virtual void draw(glm::dvec3 center);
	void adjust(double mX, double mY);
	void adjust(int crosshair, double mX, double mY);
	void detect_corners(int imgNo, double w, double h);
	~CrossHairs();
};

class CorrCrossHairs : public CrossHairs {
private:
	GLuint connecting_line_VBO[2];
	GLuint connecting_line_VAO;
	glm::mat4 proj_inv_;
	glm::mat4 cam_inv_;
	void read_input_file(const std::string file_name);

public:
	glm::vec3 color2;
	CorrCrossHairs(int totInRow, double w, double h, glm::vec3 color1, glm::vec3 color2);
	virtual void draw(glm::dvec3 center, glm::mat4 model1, glm::mat4 model2);
	virtual void init(Program* gProgram, glm::mat4 projInv, glm::mat4 camInv);
	void addPoint();
};