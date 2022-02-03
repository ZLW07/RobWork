/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "RenderText.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Math.hpp>

#ifdef RW_HAVE_GLUT
#if defined(RW_MACOS)
//#include <GLUT/glut.h>
// TODO(kalor) Figure Out how to get GLUT to work as glutBitmapString is undeclared i mac
#undef RW_HAVE_GLUT
#else
#include <GL/freeglut.h>
#endif
#endif

using namespace rwlibs::opengl;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;

namespace {
rw::math::Vector3D<> project (double x, double y, double z)
{
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    glGetDoublev (GL_MODELVIEW_MATRIX, modelMatrix);
    glGetDoublev (GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

    GLdouble winx, winy, winz;
    gluProject (x, y, z, modelMatrix, projMatrix, viewport, &winx, &winy, &winz);
    return Vector3D<> (winx, winy, winz);
}

rw::math::Vector3D<> unproject (int x, int y)
{
    GLfloat depth;
    glReadPixels (x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    glGetDoublev (GL_MODELVIEW_MATRIX, modelMatrix);
    glGetDoublev (GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv (GL_VIEWPORT, viewport);
    GLdouble objx, objy, objz;
    gluUnProject (x, y, depth, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);
    return Vector3D<> (objx, objy, objz);
}

rw::math::Vector3D<> unproject (Vector3D<> pos2D)
{
    return unproject (int (pos2D[0]), int (pos2D[1]));
}

void renderSquare (Vector3D<> pos1, Vector3D<> pos2, Vector3D<> pos3, Vector3D<> pos4,
                   std::vector< GLfloat > boxColor, std::vector< GLfloat > borderColor)
{
    glPushAttrib (GL_ALL_ATTRIB_BITS);
    glEnable (GL_BLEND);
    // Draw Square
    glPolygonMode (GL_FRONT, GL_FILL);
    if (boxColor.size () == 4u) {
        glColor4f (boxColor[0], boxColor[1], boxColor[2], boxColor[3]);
    }
    else if (boxColor.size () == 3u) {
        glColor3f (boxColor[0], boxColor[1], boxColor[2]);
    }
    else {
        glColor3f (255, 255, 255);
    }
    glColor3f (255, 255, 255);
    glEnable (GL_POLYGON_OFFSET_FILL);
    glPolygonOffset (1, 1);
    glBegin (GL_QUADS);
    glVertex3f (GLfloat (pos1[0]), GLfloat (pos1[1]), GLfloat (pos1[2]));    // vertex 1
    glVertex3f (GLfloat (pos2[0]), GLfloat (pos2[1]), GLfloat (pos2[2]));    // vertex 2
    glVertex3f (GLfloat (pos3[0]), GLfloat (pos3[1]), GLfloat (pos3[2]));    // vertex 3
    glVertex3f (GLfloat (pos4[0]), GLfloat (pos4[1]), GLfloat (pos4[2]));    // vertex 4
    glEnd ();
    glDisable (GL_POLYGON_OFFSET_FILL);

    // Draw bounding box
    glPolygonMode (GL_FRONT, GL_LINE);
    if (boxColor.size () == 4u) {
        glColor4f (borderColor[0], borderColor[1], borderColor[2], borderColor[3]);
    }
    else if (boxColor.size () == 3u) {
        glColor3f (borderColor[0], borderColor[1], borderColor[2]);
    }
    else {
        glColor3f (0, 0, 0);
    }

    glBegin (GL_QUADS);
    glVertex3f (GLfloat (pos1[0]), GLfloat (pos1[1]), GLfloat (pos1[2]));    // vertex 1
    glVertex3f (GLfloat (pos2[0]), GLfloat (pos2[1]), GLfloat (pos2[2]));    // vertex 2
    glVertex3f (GLfloat (pos3[0]), GLfloat (pos3[1]), GLfloat (pos3[2]));    // vertex 3
    glVertex3f (GLfloat (pos4[0]), GLfloat (pos4[1]), GLfloat (pos4[2]));    // vertex 4
    glEnd ();
    glDisable (GL_BLEND);
    glPopAttrib ();
}

double pixelPerMeter (Transform3D<> fTc, const Vector3D<>& pos, Vector3D<>& posCam)
{
    Transform3D< double > cTf = inverse (fTc);

    posCam          = cTf * pos;
    Vector3D<> posT = fTc * (posCam + Vector3D<> (1, 1, 0));

    Vector3D<> p2D_1 = project (pos[0], pos[1], pos[2]);
    Vector3D<> p2D_2 = project (posT[0], posT[1], posT[2]);

    return std::abs (p2D_1[0] - p2D_2[0]);
}
void moveCloserToCam (Transform3D<> fTc, Vector3D<>& pos, double distance, SceneCamera::Ptr cam)
{
    Vector3D<> diff    = fTc.P () - pos;
    double movePercent = distance / diff.norm2 ();

    // makes sure the label isn't removed due to being to close to the camera
    double zNear           = 1.02 * cam->getProjectionMatrix ().getClipPlanes ().first;
    double max_movePercent = (diff.norm2 () - zNear) / diff.norm2 ();

    if (movePercent > max_movePercent) {
        movePercent = 0.90 * max_movePercent;
    }

    // make sure the label isn't to close in general
    if (movePercent > 0.90) {
        movePercent = 0.9;
    }

    pos += diff * movePercent;
}
}    // namespace

#ifdef RW_HAVE_GLUT
#define HELVETICA_18 GLUT_BITMAP_HELVETICA_18
#define HELVETICA_12 GLUT_BITMAP_HELVETICA_12
#define HELVETICA_10 GLUT_BITMAP_HELVETICA_10
namespace {
void drawText (Vector3D<> pos, std::string text, void* font)
{
    glRasterPos3f (pos[0], pos[1], pos[2]);
    glColor4f (0, 0, 0, 1.0);
    glutBitmapString (font, (unsigned char*) text.c_str ());
}
}    // namespace
void RenderText::findTextDimensions ()
{
    _haveGlut  = true;
    _textWidth = glutBitmapLength (_font, (unsigned char*) _text.c_str ());
    _textHight = glutBitmapHeight (_font);
}
#endif
#ifndef RW_HAVE_GLUT
#define HELVETICA_18 NULL
#define HELVETICA_12 NULL
#define HELVETICA_10 NULL
namespace {
void drawText (Vector3D<> pos, std::string text, void* font)
{}
}    // namespace
void RenderText::findTextDimensions ()
{
    _haveGlut = false;
}
#endif

RenderText::RenderText (std::string text, Frame::Ptr frame) : _frame (frame), _font (HELVETICA_12)
{
    _text = " " + text + " ";
    findTextDimensions ();
}

void RenderText::draw (const DrawableNode::RenderInfo& info, DrawableNode::DrawType type,
                       double alpha) const
{
    if (_haveGlut && !info._cam.isNull ()) {
        SceneCamera::Ptr cam = info._cam;
        Transform3D<> wTf    = _frame->wTf (*(info._state));
        Transform3D<> wTc    = cam->getTransform ();
        Transform3D<> fTc    = inverse (wTf) * wTc;

        Vector3D<> pos (0, 0, 0);                // Initialize label position in frame
        moveCloserToCam (fTc, pos, 0.5, cam);    // Move the label closer to the camera to avoid,
                                                 // the 3D model to obscure the label
        drawText (pos, _text, _font);    // Draw the text

        // Get the corners for the label box
        std::vector< rw::math::Vector3D<> > corners = getLabelCorners (fTc, pos, 1, 1, 0, 0);

        // Render the label bounding box
        renderSquare (corners[0],
                      corners[1],
                      corners[2],
                      corners[3],
                      {255, 255, 255, float (alpha)},
                      {0.5, 0.5, 0.5, float (alpha)});
    }
}

std::vector< rw::math::Vector3D<> > RenderText::getLabelCorners (Transform3D<> fTc, Vector3D<> pos,
                                                                 double scale_x, double scale_y,
                                                                 int move_x, int move_y) const
{
    Vector3D<> posCam;
    double ppm    = pixelPerMeter (fTc, pos, posCam);
    double width  = scale_x * _textWidth / ppm;
    double height = scale_y * _textHight / ppm;
    double posX   = move_x / ppm;
    double posY   = (move_y - 5 * scale_y) / ppm;

    double offset = -0.00001;
    if (Vector3D<> (fTc.P () - pos).norm2 () < 0.01) {
        offset = 0;
    }
    Vector3D<> pos1 = fTc * (posCam + Vector3D<> (posX, posY, offset));
    Vector3D<> pos2 = fTc * (posCam + Vector3D<> (width + posX, posY, offset));
    Vector3D<> pos3 = fTc * (posCam + Vector3D<> (width + posX, height + posY, offset));
    Vector3D<> pos4 = fTc * (posCam + Vector3D<> (posX, height + posY, offset));
    return {pos1, pos2, pos3, pos4};
}
