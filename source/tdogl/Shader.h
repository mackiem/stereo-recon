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

#include <GL/glew.h>
#include <string>

namespace tdogl {

    /**
     Represents a compiled OpenGL shader.
     */
    class Shader { 
    public:
        
        /**
         Creates a shader from a text file.
         
         @param filePath    The path to the text file containing the shader source.
         @param shaderType  Same as the argument to glCreateShader. For example GL_VERTEX_SHADER
                            or GL_FRAGMENT_SHADER.
         
         @throws std::exception if an error occurs.
         */
        static Shader shaderFromFile(const std::string& filePath, GLenum shaderType);
        
        
        /**
         Creates a shader from a string of shader source code.
         
         @param shaderCode  The source code for the shader.
         @param shaderType  Same as the argument to glCreateShader. For example GL_VERTEX_SHADER
                            or GL_FRAGMENT_SHADER.
         
         @throws std::exception if an error occurs.
         */
        Shader(const std::string& shaderCode, GLenum shaderType);
        
        
        /**
         @result The shader's object ID, as returned from glCreateShader
         */
        GLuint object() const;
        
        // tdogl::Shader objects can be copied and assigned because they are reference counted
        // like a shared pointer
        Shader(const Shader& other);
        Shader& operator =(const Shader& other);
        ~Shader();
        
    private:
        GLuint _object;
        unsigned* _refCount;
        
        void _retain();
        void _release();
    };
    
}
