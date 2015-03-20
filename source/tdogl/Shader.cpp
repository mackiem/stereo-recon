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
#include "Shader.h"
#include <stdexcept>
#include <fstream>
#include <string>
#include <cassert>
#include <sstream>

using namespace tdogl;

Shader::Shader(const std::string& shaderCode, GLenum shaderType) :
    _object(0),
    _refCount(NULL)
{
    //create the shader object
    _object = glCreateShader(shaderType);
    if(_object == 0)
        throw std::runtime_error("glCreateShader failed");
    
    //set the source code
    const char* code = shaderCode.c_str();
    glShaderSource(_object, 1, (const GLchar**)&code, NULL);
    
    //compile
    glCompileShader(_object);
    
    //throw exception if compile error occurred
    GLint status;
    glGetShaderiv(_object, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        std::string msg("Compile failure in shader:\n");
        
        GLint infoLogLength;
        glGetShaderiv(_object, GL_INFO_LOG_LENGTH, &infoLogLength);
        char* strInfoLog = new char[infoLogLength + 1];
        glGetShaderInfoLog(_object, infoLogLength, NULL, strInfoLog);
        msg += strInfoLog;
        delete[] strInfoLog;
        
        glDeleteShader(_object); _object = 0;
        throw std::runtime_error(msg);
    }
    
    _refCount = new unsigned;
    *_refCount = 1;
}

Shader::Shader(const Shader& other) :
    _object(other._object),
    _refCount(other._refCount)
{
    _retain();
}

Shader::~Shader() {
    //_refCount will be NULL if constructor failed and threw an exception
    if(_refCount) _release();
}

GLuint Shader::object() const {
    return _object;
}

Shader& Shader::operator = (const Shader& other) {
    _release();
    _object = other._object;
    _refCount = other._refCount;
    _retain();
    return *this;
}

Shader Shader::shaderFromFile(const std::string& filePath, GLenum shaderType) {
    //open file
    std::ifstream f;
    f.open(filePath.c_str(), std::ios::in | std::ios::binary);
    if(!f.is_open()){
        throw std::runtime_error(std::string("Failed to open file: ") + filePath);
    }

    //read whole file into stringstream buffer
    std::stringstream buffer;
    buffer << f.rdbuf();

    //return new shader
    Shader shader(buffer.str(), shaderType);
    return shader;
}

void Shader::_retain() {
    assert(_refCount);
    *_refCount += 1;
}

void Shader::_release() {
    assert(_refCount && *_refCount > 0);
    *_refCount -= 1;
    if(*_refCount == 0){
        glDeleteShader(_object); _object = 0;
        delete _refCount; _refCount = NULL;
    }
}

