/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DRAW_H
#define DRAW_H
#include <cstdlib>
#include <GL/glut.h>
#include <math.h>
#include <iostream>
#include <string>
#include <control_node.h>


void drawEllipse(float xradius, float yradius);

void drawTree(BT::ControlNode* tree_);

void draw_status(float x, float y, int node_status);

void drawString(void * font, const char *string, float x, float y, float z);

void renderBitmapString(float x, float y, void *font, const char *string);

void draw_node(float x, float y, int node_type, const char *leafName, int status);

void draw_edge(GLfloat parent_x, GLfloat parent_y, GLfloat parent_size,
               GLfloat child_x, GLfloat child_y, GLfloat child_size);

void keyboard(unsigned char key, int x, int y);

void drawCircle(float radius);

int compute_node_lines(const char *string);

int compute_max_width(const char *string);

#endif  // DRAW_H
