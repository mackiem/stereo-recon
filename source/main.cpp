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

// third-party libraries
#include <Windows.h>
#include <GL/glew.h>
#include <GL/glfw.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// standard C++ libraries
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

// tdogl classes
#include "tdogl/Program.h"
#include "tdogl/Texture.h"
#include "crosshairs.h"
#include "calibration.h"
#include "recon.h"

#define GLFW_CDECL
#include <AntTweakBar.h>


// constants
const glm::vec2 SCREEN_SIZE(1280, 800);

// globals
tdogl::Texture* gTexture = NULL;
tdogl::Texture* gTexture2 = NULL;
tdogl::Texture* gTexture3 = NULL;
tdogl::Program* gProgram = NULL;
GLuint gVAO = 0;
GLuint gVBO = 0;
GLfloat gDegreesRotated = 0.0f;
CrossHairs* crossHairs;

CorrCrossHairs* corr_cross_hairs;
glm::dvec3 center;

// states
static int sDrawCrossHairs = 0;
static int sFitCrossHairs = 0;
static int sCalibrate = 0;


// returns the full path to the file `fileName` in the resources directory of the app bundle
static std::string ResourcePath(std::string fileName) {
    char executablePath[1024] = {'\0'};
    DWORD charsCopied = GetModuleFileName(NULL, executablePath, 1024);
    if(charsCopied > 0 && charsCopied < 1024)
        return std::string(executablePath) + "\\..\\" + fileName;
    else
        throw std::runtime_error("GetModuleFileName failed a bit");
}

glm::mat4 invProj;
glm::mat4 invCam;
const float offset = 1.0f;

// loads the vertex shader and fragment shader, and links them to make the global gProgram
static void LoadShaders() {
    std::vector<tdogl::Shader> shaders;
    shaders.push_back(tdogl::Shader::shaderFromFile(ResourcePath("vertex-shader.txt"), GL_VERTEX_SHADER));
    shaders.push_back(tdogl::Shader::shaderFromFile(ResourcePath("fragment-shader.txt"), GL_FRAGMENT_SHADER));
    gProgram = new tdogl::Program(shaders);

    gProgram->use();

    //set the "projection" uniform in the vertex shader, because it's not going to change
    glm::mat4 projection = glm::perspective<float>(45.0, SCREEN_SIZE.x/SCREEN_SIZE.y, 0.1, 10.0);
    //glm::mat4 projection = glm::ortho<float>(-2, 2, -2, 2, 0.1, 10);
    gProgram->setUniform("projection", projection);
	invProj = projection._inverse();

    //set the "camera" uniform in the vertex shader, because it's also not going to change
	center = glm::vec3(0,0,0);
    glm::mat4 camera = glm::lookAt(glm::vec3(0,0, 3.5f), glm::vec3(center), glm::vec3(0,1,0));
    gProgram->setUniform("camera", camera);
	invCam = projection._inverse();

    // cross hairs
	crossHairs = new CrossHairs(5, SCREEN_SIZE.x, SCREEN_SIZE.y, glm::vec3(0.f, 1.f, 0.f));
	crossHairs->init(gProgram);
	glLineWidth(3.f);
	//crossHairs->detect_corners(1, SCREEN_SIZE.x, SCREEN_SIZE.y);
	//crossHairs->detect_corners(2, SCREEN_SIZE.x, SCREEN_SIZE.y);
	//crossHairs->detect_corners(3, SCREEN_SIZE.x, SCREEN_SIZE.y);

	corr_cross_hairs = new CorrCrossHairs(5, SCREEN_SIZE.x, SCREEN_SIZE.y, glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 1.f, 1.f));
	corr_cross_hairs->init(gProgram, invProj, invCam);

    gProgram->stopUsing();
}

#define PI 3.14159265359
static int isCorr = 1;

//static void handleMousePress() {

void GLFWCALL OnMousePress(int button, int action)  // your callback function called by GLFW when mouse has moved
{
	if (!TwEventMouseButtonGLFW(button, action)) {
		if ((button == GLFW_MOUSE_BUTTON_LEFT) && (action == GLFW_PRESS)) {

			int x, y;
			double w = SCREEN_SIZE.x;
			double h = SCREEN_SIZE.y;

			glfwGetMousePos(&x, &y);
			double cX = x - (w / 2.0);
			double iW = tan(22.5f * (double)PI / 180) * 0.1 * w / h;
			double oW = (3.5f / 0.1) * iW;
			double mX = oW * cX / (w / 2.0);

			double cY = y - (h / 2.0);
			double oY = oW * h / w;
			double mY = oY * cY / (h / 2.0);


			//float mX = (((float)2 * x) - w) / w;
			//float mX = (0.1f + 3.5f) * x / 0.1f;
			//   float mY = (0.1f + 3.5f) * y / 0.1f;
			//mY = (((float)2 * mY) - h) / h;
			if (isCorr == 0) {
			    crossHairs->adjust(mX, -mY);
			}
			else {
			    corr_cross_hairs->adjust(mX, -mY);
			}
			//glm::vec4 pos(x, y, 0.f, 1.f);
			//glm::vec4 adjM = pos * invProj * invCam;
			//crossHairs->adjust(mX, -mY);
			//crossHairs->adjust(adjM.x, adjM.y);
			std::cout << x << ", " << y << std::endl;
			std::cout << mX << ", " << -mY << std::endl;
		}
	}
}

static bool sortx(glm::dvec2 a, glm::dvec2 b) {
	const float t = 1e-10;
	return (a.x < b.x);
}

static bool sortxy(glm::dvec2 a, glm::dvec2 b) {
	const float t = 0.7e-1;
	if ((a.x < b.x + t) && (a.x > b.x - t)) {
		return a.y < b.y;
	}
	return (a.x < b.x);
}

static CrossHairs* redCrossHairs;
static Calibration calib;
static int imgNo = 1;
static int calibrated = 0;
static TwBar* bar;
static int corrNo = 1;
static Recon recon;
static int is_recon = 0;

static void calibrate() {
		calib.calibrate();
	    redCrossHairs = new CrossHairs(5, SCREEN_SIZE.x, SCREEN_SIZE.y, glm::vec3(1.f, 0.f, 0.f));
	    redCrossHairs->init(gProgram);
		calib.posCrossHairs(redCrossHairs, imgNo);
		calibrated = 1;
}

static void reconstruct() {
	recon.set_gprogram(gProgram);
	recon.calibrate();
	recon.reconstruct_obj(corrNo, &calib);
	is_recon = 1;
}


static void saveImg() {
		std::vector<glm::dvec2> imgPoints;
		for (auto& mapItr = crossHairs->pos.begin(); mapItr != crossHairs->pos.end(); ++mapItr) {
			glm::dvec3 pt = mapItr->second;
			imgPoints.push_back(glm::dvec2(pt.x, pt.y));
		}
		std::sort(imgPoints.begin(), imgPoints.end(), sortx);
		std::sort(imgPoints.begin(), imgPoints.end(), sortxy);
		std::vector<glm::dvec3> worldPts;
		//const double width = 101.6; // mm, 4 inches
		//const double width = 76.2; // mm, 3 inches
		const double width = 38.1; // mm, 1.5 inches
		const double height = width; // square
		for (int i = 0; i < crossHairs->totInRow; ++i) {
			for (int j = 0; j < crossHairs->totInRow; ++j) {
				worldPts.push_back(glm::dvec3(i * width, j * height, 0.f));
			}
		}
		calib.saveImgPnts(worldPts, imgPoints, imgNo);
}

static void load_img_points() {
		calib.load_img_points(crossHairs, imgNo);
}

static void save_corr() {
		std::vector<glm::dvec2> img_points1;
		std::vector<glm::dvec2> img_points2;
		auto& pos = corr_cross_hairs->pos;
		glm::mat4 model1 = glm::translate(glm::mat4(1.f), glm::vec3(offset, 0.f, 0.f));
		glm::mat4 model1Inv = model1._inverse();
		glm::mat4 model2 = glm::translate(glm::mat4(1.f), glm::vec3(-offset, 0.f, 0.f));
		glm::mat4 model2Inv = model2._inverse();
		for (auto i = 0u; i < pos.size(); ++i)  {
			const int h = SCREEN_SIZE.y;
			const int w = SCREEN_SIZE.x;
			glm::vec4 modelPos = model1Inv * glm::vec4(pos[i], 1.0);
			std::cout << modelPos.x << ", " << modelPos.y << std::endl;
			if (!(i % 2)) {
			    img_points1.push_back(glm::dvec2(pos[i].x, pos[i].y));
			} else {
			    img_points2.push_back(glm::dvec2(pos[i].x, pos[i].y));
			}
		}
		//calib.saveImgPnts(worldPts, img_points1, imgNo);
}

static void addCorrPnt() {
	if (!isCorr) return;
}

static void detect_features() {
	recon.detect_features();
}

static void snap_to_corners() {
	crossHairs->detect_corners(imgNo, SCREEN_SIZE.x, SCREEN_SIZE.y);
}

//static void saveImg2() {
//		std::vector<glm::dvec2> imgPoints;
//		for (auto& mapItr = crossHairs->pos.begin(); mapItr != crossHairs->pos.end(); ++mapItr) {
//			glm::dvec3 pt = mapItr->second;
//			imgPoints.push_back(glm::dvec2(pt.x, pt.y));
//		}
//		std::sort(imgPoints.begin(), imgPoints.end(), sortx);
//		std::sort(imgPoints.begin(), imgPoints.end(), sortxy);
//		std::vector<glm::dvec3> worldPts;
//		const double width = 101.6; // mm, 4 inches
//		const double height = width; // square
//		for (int i = 0; i < crossHairs->totInRow; ++i) {
//			for (int j = 0; j < crossHairs->totInRow; ++j) {
//				worldPts.push_back(glm::dvec3(i * width, j * height, 0.f));
//			}
//		}
//		calib.saveImgPnts(worldPts, imgPoints, 2);
//
//}
static void load_common() {
	is_recon = 0;
	isCorr = 0;
	calib.load_img_points(crossHairs, imgNo);
}

static void loadImage1() {
	imgNo = 1;
	load_common();
}

static void loadImage2() {
	imgNo = 2;
	load_common();
}

static void loadImage3() {
	imgNo = 3;
	load_common();
}

static void loadCorr1() {
	is_recon = 0;
	isCorr = 1;
	corrNo = 1;
}

static void loadCorr2() {
	is_recon = 0;
	isCorr = 1;
	corrNo = 2;
}

static void TW_CALL calibrateAnt(void* clientData) {
	calibrate();
}

static void TW_CALL recon_ant(void* clientData) {
	reconstruct();
}

static void TW_CALL loadCorr1Ant(void* clientData) {
	loadCorr1();
}

static void TW_CALL loadCorr2Ant(void* clientData) {
	loadCorr2();
}

//static void TW_CALL addCorrPntAnt(void* clientData) {
//	addCorrPoint();
//}

static void TW_CALL loadImage1Ant(void* clientData) {
	loadImage1();
}

static void TW_CALL loadImage2Ant(void* clientData) {
	loadImage2();
}

static void TW_CALL loadImage3Ant(void* clientData) {
	loadImage3();
}

static void TW_CALL saveImg1Ant(void* clientData) {
	saveImg();
}

static void TW_CALL saveImg2Ant(void* clientData) {
	saveImg();
}

static void TW_CALL saveImg3Ant(void* clientData) {
	saveImg();
}

static void TW_CALL saveCorr1Ant(void* clientData) {
	save_corr();
}

static void TW_CALL saveCorr2Ant(void* clientData) {
	save_corr();
}

static void TW_CALL detect_features_Ant(void* clientData) {
	detect_features();
}

static void TW_CALL snap_to_corners_Ant(void* clientData) {
	snap_to_corners();
}

//static void handleKeyBoard() {
//	if (glfwGetKey('C') == GLFW_PRESS) {
//		calibrate();
//		//calib.calibrate(worldPts, imgPoints);
//	} else if (glfwGetKey('1') == GLFW_PRESS) {
//		saveImg1();
//	} else if (glfwGetKey('2') == GLFW_PRESS) {
//		saveImg2();
//	} else if (glfwGetKey('0') == GLFW_PRESS) {
//		loadImage1();
//	}
//	else if (glfwGetKey('9') == GLFW_PRESS) {
//		loadImage2();
//	}
//}

// loads a cube into the VAO and VBO globals: gVAO and gVBO
static void LoadCube() {
    // make and bind the VAO
    glGenVertexArrays(1, &gVAO);
    glBindVertexArray(gVAO);
    
    // make and bind the VBO
    glGenBuffers(1, &gVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gVBO);
    
    // Make a cube out of triangles (two triangles per side)
    GLfloat vertexData[] = {
        //  X     Y     Z       U     V
        // bottom
        //-1.0f,-1.0f,-1.0f,   0.0f, 0.0f,
        // 1.0f,-1.0f,-1.0f,   1.0f, 0.0f,
        //-1.0f,-1.0f, 1.0f,   0.0f, 1.0f,
        // 1.0f,-1.0f,-1.0f,   1.0f, 0.0f,
        // 1.0f,-1.0f, 1.0f,   1.0f, 1.0f,
        //-1.0f,-1.0f, 1.0f,   0.0f, 1.0f,

        //// top
        //-1.0f, 1.0f,-1.0f,   0.0f, 0.0f,
        //-1.0f, 1.0f, 1.0f,   0.0f, 1.0f,
        // 1.0f, 1.0f,-1.0f,   1.0f, 0.0f,
        // 1.0f, 1.0f,-1.0f,   1.0f, 0.0f,
        //-1.0f, 1.0f, 1.0f,   0.0f, 1.0f,
        // 1.0f, 1.0f, 1.0f,   1.0f, 1.0f,

        //// front
        //-1.0f,-1.0f, 1.0f,   1.0f, 0.0f,
        // 1.0f,-1.0f, 1.0f,   0.0f, 0.0f,
        //-1.0f, 1.0f, 1.0f,   1.0f, 1.0f,
        // 1.0f,-1.0f, 1.0f,   0.0f, 0.0f,
        // 1.0f, 1.0f, 1.0f,   0.0f, 1.0f,
        //-1.0f, 1.0f, 1.0f,   1.0f, 1.0f,

        // back
        -1.0f,-1.0f, 0.f,   0.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,   0.0f, 1.0f,
         1.0f,-1.0f, 0.0f,   1.0f, 0.0f,
         1.0f,-1.0f, 0.0f,   1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,   0.0f, 1.0f,
         1.0f, 1.0f, 0.0f,   1.0f, 1.0f,

        //-100.0f,-100.0f, 0.f,   0.0f, 0.0f,
        //-100.0f, 100.0f, 0.0f,   0.0f, 1.0f,
        // 100.0f,-100.0f, 0.0f,   1.0f, 0.0f,
        // 100.0f,-100.0f, 0.0f,   1.0f, 0.0f,
        //-100.0f, 100.0f, 0.0f,   0.0f, 1.0f,
        // 100.0f, 100.0f, 0.0f,   1.0f, 1.0f,

        // left
        //-1.0f,-1.0f, 1.0f,   0.0f, 1.0f,
        //-1.0f, 1.0f,-1.0f,   1.0f, 0.0f,
        //-1.0f,-1.0f,-1.0f,   0.0f, 0.0f,
        //-1.0f,-1.0f, 1.0f,   0.0f, 1.0f,
        //-1.0f, 1.0f, 1.0f,   1.0f, 1.0f,
        //-1.0f, 1.0f,-1.0f,   1.0f, 0.0f,

        //// right
        // 1.0f,-1.0f, 1.0f,   1.0f, 1.0f,
        // 1.0f,-1.0f,-1.0f,   1.0f, 0.0f,
        // 1.0f, 1.0f,-1.0f,   0.0f, 0.0f,
        // 1.0f,-1.0f, 1.0f,   1.0f, 1.0f,
        // 1.0f, 1.0f,-1.0f,   0.0f, 0.0f,
        // 1.0f, 1.0f, 1.0f,   0.0f, 1.0f
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);

    // connect the xyz to the "vert" attribute of the vertex shader
    glEnableVertexAttribArray(gProgram->attrib("vert"));
    glVertexAttribPointer(gProgram->attrib("vert"), 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), NULL);
        
    // connect the uv coords to the "vertTexCoord" attribute of the vertex shader
    glEnableVertexAttribArray(gProgram->attrib("vertTexCoord"));
    glVertexAttribPointer(gProgram->attrib("vertTexCoord"), 2, GL_FLOAT, GL_TRUE,  5*sizeof(GLfloat), (const GLvoid*)(3 * sizeof(GLfloat)));

    // unbind the VAO
    glBindVertexArray(0);
}



// loads the file "wooden-crate.jpg" into gTexture
static void LoadTexture() {
    //tdogl::Bitmap bmp = tdogl::Bitmap::bitmapFromFile(ResourcePath("wooden-crate.jpg"));
    tdogl::Bitmap bmp = tdogl::Bitmap::bitmapFromFile(ResourcePath("3dr1.jpg"));
    tdogl::Bitmap bmp2 = tdogl::Bitmap::bitmapFromFile(ResourcePath("3dr2.jpg"));
    tdogl::Bitmap bmp3 = tdogl::Bitmap::bitmapFromFile(ResourcePath("3dr3.jpg"));
    bmp.flipVertically();
    bmp2.flipVertically();
    bmp3.flipVertically();
    gTexture = new tdogl::Texture(bmp);
    gTexture2 = new tdogl::Texture(bmp2);
    gTexture3 = new tdogl::Texture(bmp3);
}


// draws a single frame
static void Render() {
    // clear everything
    glClearColor(0, 0, 0, 1); // black
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//handleMousePress();
	//handleKeyBoard();
    // bind the program (the shaders)
    gProgram->use();

    // set the "model" uniform in the vertex shader, based on the gDegreesRotated global
    //gProgram->setUniform("model", glm::rotate(glm::mat4(), gDegreesRotated, glm::vec3(0,1,0)));
        
    // bind the texture and set the "tex" uniform in the fragment shader
    glActiveTexture(GL_TEXTURE0);

	if (is_recon) {
		recon.draw();
	}
	else {

		if (isCorr == 0) {
			if (imgNo == 1) {
				glBindTexture(GL_TEXTURE_2D, gTexture->object());
			}
			else if (imgNo == 2) {
				glBindTexture(GL_TEXTURE_2D, gTexture2->object());
			}
			else if (imgNo == 3) {
				glBindTexture(GL_TEXTURE_2D, gTexture3->object());
			}

			if (calibrated == 1) {
				redCrossHairs->draw(center);
			}
			gProgram->setUniform("tex", 0); //set to 0 because the texture is bound to GL_TEXTURE0

			crossHairs->draw(center);
			// bind the VAO (the triangle)
			glBindVertexArray(gVAO);

			gProgram->setUniform("model", glm::mat4(1.f));
			// draw the VAO
			glDrawArrays(GL_TRIANGLES, 0, 6);

			// unbind the VAO, the program and the texture
			glBindVertexArray(0);
			glBindTexture(GL_TEXTURE_2D, 0);

			if (calibrated == 1) {
				redCrossHairs->draw(center);
			}
			crossHairs->draw(center);
		}
		else {
			glm::mat4 model1 = glm::translate(glm::mat4(1.f), glm::vec3(offset, 0.f, 0.f));
			glm::mat4 model2 = glm::translate(glm::mat4(1.f), glm::vec3(-offset, 0.f, 0.f));
			corr_cross_hairs->draw(center, model1, model2);
			if (corrNo == 1) {
				glBindVertexArray(gVAO);
				glBindTexture(GL_TEXTURE_2D, gTexture->object());
				gProgram->setUniform("tex", 0); //set to 0 because the texture is bound to GL_TEXTURE0

				// bind the VAO (the triangle)

				gProgram->setUniform("model", model1);
				// draw the VAO
				glDrawArrays(GL_TRIANGLES, 0, 6);

				glBindTexture(GL_TEXTURE_2D, gTexture2->object());
				gProgram->setUniform("model", model2);
				glDrawArrays(GL_TRIANGLES, 0, 6);
				glBindVertexArray(0);
			}
			else if (corrNo == 2) {
				glBindVertexArray(gVAO);
				glBindTexture(GL_TEXTURE_2D, gTexture2->object());
				gProgram->setUniform("tex", 0); //set to 0 because the texture is bound to GL_TEXTURE0

				// bind the VAO (the triangle)

				gProgram->setUniform("model", glm::translate(glm::mat4(1.f), glm::vec3(offset, 0.f, 0.f)));
				// draw the VAO
				glDrawArrays(GL_TRIANGLES, 0, 6);

				glBindTexture(GL_TEXTURE_2D, gTexture3->object());
				gProgram->setUniform("model", glm::translate(glm::mat4(1.f), glm::vec3(-offset, 0.f, 0.f)));
				glDrawArrays(GL_TRIANGLES, 0, 6);
				glBindVertexArray(0);
			}


		}
	}


		// unbind the VAO, the program and the texture
		glBindTexture(GL_TEXTURE_2D, 0);

    // draw cross hairs


    gProgram->stopUsing();
    
	//glfwPollEvents();
	TwDraw();
    // swap the display buffers (displays what was just drawn)
    glfwSwapBuffers();
}


// update the scene based on the time elapsed since last update
void Update(float secondsElapsed) {
    const GLfloat degreesPerSecond = 180.0f;
    gDegreesRotated += secondsElapsed * degreesPerSecond;
    while(gDegreesRotated > 360.0f) gDegreesRotated -= 360.0f;
}

// the program starts here
void AppMain() {
    // initialise GLFW
    if(!glfwInit())
        throw std::runtime_error("glfwInit failed");
    
    // open a window with GLFW
    glfwOpenWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, 3);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, 2);
    glfwOpenWindowHint(GLFW_WINDOW_NO_RESIZE, GL_TRUE);
    if(!glfwOpenWindow((int)SCREEN_SIZE.x, (int)SCREEN_SIZE.y, 8, 8, 8, 8, 16, 0, GLFW_WINDOW))
        throw std::runtime_error("glfwOpenWindow failed. Can your hardware handle OpenGL 3.2?");
    
    // initialise GLEW
    glewExperimental = GL_TRUE; //stops glew crashing on OSX :-/
    if(glewInit() != GLEW_OK)
        throw std::runtime_error("glewInit failed");
    
    // GLEW throws some errors, so discard all the errors so far
    while(glGetError() != GL_NO_ERROR) {}

    // print out some info about the graphics drivers
    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    std::cout << "Vendor: " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;

    // make sure OpenGL version 3.2 API is available
    if(!GLEW_VERSION_3_2)
        throw std::runtime_error("OpenGL 3.2 API is not available.");

    // OpenGL settings
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Ant Bar

	TwWindowSize(SCREEN_SIZE.x, SCREEN_SIZE.y);
	// Ant Tweak Bar
	TwInit(TW_OPENGL_CORE, NULL);
	bar = TwNewBar("CC");
	TwDefine(" GLOBAL help='This example shows how to integrate AntTweakBar with GLFW and OpenGL.' "); // Message added to the help bar.

	//TwAddButton(bar, "Load Image 1", loadImage1Ant, NULL, " label='Load Image 1' key=9 group='Calibrate'");
	//TwAddButton(bar, "Save Image 1 Points", saveImg1Ant, NULL, " label='Save Image 1' key=1 group='Save'");
	//TwAddButton(bar, "Load Image 2", loadImage2Ant, NULL, " label='Load Image 2' key=0 group='Calibrate'");
	//TwAddButton(bar, "Save Image 2 Points", saveImg2Ant, NULL, " label='Save Image 2' key=2 group='Save'");
	//TwAddButton(bar, "Load Image 3", loadImage3Ant, NULL, " label='Load Image 3' key=- group='Calibrate'");
	//TwAddButton(bar, "Save Image 3 Points", saveImg3Ant, NULL, " label='Save Image 3' key=3 group='Save'");
	TwAddButton(bar, "Load Corr 1 and 2", loadCorr1Ant, NULL, " label='Load Image 1 & 2' key=c group='Correspond'");
	//TwAddButton(bar, "Save Corr Points", saveCorr1Ant, NULL, " label='Save Corr Points' key=5 group='Correspond'");
	TwAddButton(bar, "Feature Detect", detect_features_Ant, NULL, " label='Detect Features' key=d group='Correspond'");
	//TwAddButton(bar, "Snap to corners", snap_to_corners_Ant, NULL, " label='Snap to Corners' key=d group='Calibrate'");

	//TwAddButton(bar, "Calibratee", calibrateAnt, NULL, " label='Calibrate' key=z group='Calibrate'");
	TwAddButton(bar, "Reconstruct", recon_ant, NULL, " label='Reconstruct' key=r group='Recon'");
	//TwAddButton(bar, "add Corr poitn", addPnt, NULL, " label='Add Corr. Point' key=a group='Correspond'");

	//glfwSetMouseButtonCallback((GLFWmousebuttonfun)TwEventMouseButtonGLFW);
	// - Directly redirect GLFW mouse position events to AntTweakBar
	glfwSetMousePosCallback((GLFWmouseposfun)TwEventMousePosGLFW);
	// - Directly redirect GLFW mouse wheel events to AntTweakBar
	glfwSetMouseWheelCallback((GLFWmousewheelfun)TwEventMouseWheelGLFW);
	// - Directly redirect GLFW key events to AntTweakBar
	glfwSetKeyCallback((GLFWkeyfun)TwEventKeyGLFW);
	//// - Directly redirect GLFW char events to AntTweakBar
	glfwSetCharCallback((GLFWcharfun)TwEventCharGLFW);

	glfwSetMouseButtonCallback(OnMousePress);


    // load vertex and fragment shaders into opengl
    LoadShaders();

    // load the texture
    LoadTexture();

    // create buffer and fill it with the points of the triangle
    LoadCube();

    // run while the window is open
    double lastTime = glfwGetTime();
    while(glfwGetWindowParam(GLFW_OPENED)){
        // update the scene based on the time elapsed since last update
        double thisTime = glfwGetTime();
        Update(thisTime - lastTime);
        lastTime = thisTime;
        
        // draw one frame
        Render();

        // check for errors
        GLenum error = glGetError();
        if(error != GL_NO_ERROR)
            std::cerr << "OpenGL Error " << error << ": " << (const char*)gluErrorString(error) << std::endl;
    }

    // clean up and exit
    glfwTerminate();
}


int main(int argc, char *argv[]) {
    try {
        AppMain();
    } catch (const std::exception& e){
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
