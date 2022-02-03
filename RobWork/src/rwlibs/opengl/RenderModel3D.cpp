#include "RenderModel3D.hpp"

#include "DrawableUtil.hpp"
#include "RWGLTexture.hpp"

#include <rw/graphics/SceneCamera.hpp>

using namespace rw::math;

using namespace rwlibs::opengl;
using namespace rw::graphics;

RenderModel3D::RenderModel3D (Model3D::Ptr model) : _model (model)
{
    // create list of textures
    for (std::size_t i = 0; i < _model->getTextures<Model3DTextureType>().size (); i++) {
        _textures.push_back (rw::core::ownedPtr (new RWGLTexture ()));
    }
}

RenderModel3D::~RenderModel3D ()
{}

void RenderModel3D::draw (const DrawableNode::RenderInfo& info, DrawableNode::DrawType type,
                          double alpha) const
{
    glPushAttrib (GL_ALL_ATTRIB_BITS);
    switch (type) {
        case DrawableNode::SOLID:
            glPolygonMode (GL_FRONT, GL_FILL);

            drawUsingSimple (info, type, alpha);

            break;
        case DrawableNode::OUTLINE:    // Draw nice frame
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
            glEnable (GL_POLYGON_OFFSET_FILL);
            glPolygonOffset (1, 1);
            drawUsingSimple (info, type, alpha);
            glDisable (GL_POLYGON_OFFSET_FILL);

            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
            glColor3f (0.0f, 0.0f, 0.0f);
            drawUsingSimple (info, type, alpha, true);

            break;
        case DrawableNode::WIRE:
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
            drawUsingSimple (info, type, alpha);

            break;
    }
    glPopAttrib ();
}
namespace rwlibs { namespace opengl {
    struct TPFace
    {
        size_t _mat;
        std::vector< const float* > _texCord;
        std::vector< const float* > _normal;
        std::vector< const float* > _vertex;
        rw::math::Transform3D< float > _transform;
        double dist2cam;
        bool operator< (const TPFace& lhs) { return dist2cam < lhs.dist2cam; }
        bool operator< (const TPFace& lhs) const { return dist2cam < lhs.dist2cam; }
    };
}}    // namespace rwlibs::opengl

template< class T >
void RenderModel3D::makeVertexList (const Model3D::Object3D< T >& obj, std::vector< TPFace >& list,
                                    Transform3D< float > initial) const
{
    Transform3D< float > trans = initial * obj._transform;
    if (obj._normals.size () != 0 && obj._vertices.size () != 0) {
        // Loop through the faces as sorted by material and draw them
        for (const typename Model3D::Object3DGeneric::MaterialMapData& data : obj._materialMap) {
            TPFace tmpTPFace;
            tmpTPFace._transform = trans;
            tmpTPFace._mat       = data.matId;

            if (_model->_materials[data.matId].hasTexture ()) {
                if (obj._mappedToFaces) {
                    for (std::size_t i = data.startIdx; i < data.startIdx + data.size; i++) {
                        TPFace face                                   = tmpTPFace;
                        const rw::geometry::IndexedTriangle< T >& tri = obj._faces[i];

                        face._texCord.push_back (&obj._texCoords[i * 3](0));
                        face._normal.push_back (&obj._normals[tri[0]](0));
                        face._vertex.push_back (&obj._vertices[tri[0]](0));

                        face._texCord.push_back (&obj._texCoords[i * 3 + 1](0));
                        face._normal.push_back (&obj._normals[tri[1]](0));
                        face._vertex.push_back (&obj._vertices[tri[1]](0));

                        face._texCord.push_back (&obj._texCoords[i * 3 + 2](0));
                        face._normal.push_back (&obj._normals[tri[2]](0));
                        face._vertex.push_back (&obj._vertices[tri[2]](0));

                        list.push_back (face);
                    }
                }
                else {
                    for (std::size_t i = data.startIdx; i < data.startIdx + data.size; i++) {
                        TPFace face                                   = tmpTPFace;
                        const rw::geometry::IndexedTriangle< T >& tri = obj._faces[i];

                        face._texCord.push_back (&obj._texCoords[tri[0]](0));
                        face._normal.push_back (&obj._normals[tri[0]](0));
                        face._vertex.push_back (&obj._vertices[tri[0]](0));

                        face._texCord.push_back (&obj._texCoords[tri[1]](0));
                        face._normal.push_back (&obj._normals[tri[1]](0));
                        face._vertex.push_back (&obj._vertices[tri[1]](0));

                        face._texCord.push_back (&obj._texCoords[tri[2]](0));
                        face._normal.push_back (&obj._normals[tri[2]](0));
                        face._vertex.push_back (&obj._vertices[tri[2]](0));

                        list.push_back (face);
                    }
                }
            }
            else {
                for (std::size_t i = data.startIdx; i < data.startIdx + data.size; i++) {
                    TPFace face                                   = tmpTPFace;
                    const rw::geometry::IndexedTriangle< T >& tri = obj._faces[i];

                    face._normal.push_back (&obj._normals[tri[0]](0));
                    face._vertex.push_back (&obj._vertices[tri[0]](0));

                    face._normal.push_back (&obj._normals[tri[1]](0));
                    face._vertex.push_back (&obj._vertices[tri[1]](0));

                    face._normal.push_back (&obj._normals[tri[2]](0));
                    face._vertex.push_back (&obj._vertices[tri[2]](0));

                    list.push_back (face);
                }
            }
        }
    }

    for (const Model3D::Object3DGeneric::Ptr& child : obj._kids) {
        if (const Model3D::Object3D< uint8_t >::Ptr objPtrT =
                child.cast< Model3D::Object3D< uint8_t > > ()) {
            makeVertexList< uint8_t > (*objPtrT, list, trans);
        }
        else if (const Model3D::Object3D< uint16_t >::Ptr objPtrT =
                     child.cast< Model3D::Object3D< uint16_t > > ()) {
            makeVertexList< uint16_t > (*objPtrT, list, trans);
        }
        else if (const Model3D::Object3D< uint32_t >::Ptr objPtrT =
                     child.cast< Model3D::Object3D< uint32_t > > ()) {
            makeVertexList< uint32_t > (*objPtrT, list, trans);
        }
        else {
            RW_THROW ("RenderModel3D could not recognize the type of Object3D in the model.");
        }
    }
}

void RenderModel3D::drawTPFaceList (const std::vector< TPFace >& list,
                                    const DrawableNode::RenderInfo& info,
                                    rw::graphics::DrawableNode::DrawType type, double alpha,
                                    bool disableMaterials) const
{
    glPushMatrix ();

    for (size_t i = 0; i < list.size (); i++) {
        glPushMatrix ();
        DrawableUtil::multGLTransform (list[i]._transform);
        size_t j;
        for (j = i; j < list.size (); j++) {
            if (list[i]._transform != list[j]._transform) {
                break;
            }
            glPushMatrix ();

            if (!disableMaterials) {
                useMaterial (_model->_materials[list[j]._mat], type, alpha);
            }
            if (_model->_materials[list[j]._mat].hasTexture ()) {
                glEnable (GL_TEXTURE_2D);
            }
            size_t k;
            for (k = j; k < list.size (); k++) {
                if (list[i]._transform != list[k]._transform) {
                    break;
                }
                if (list[k]._mat != list[j]._mat && !disableMaterials) {
                    break;
                }

                glBegin (GL_TRIANGLES);
                if (!list[k]._texCord.empty ()) {
                    glTexCoord2fv (list[k]._texCord[0]);
                }
                glNormal3fv (list[k]._normal[0]);
                glVertex3fv (list[k]._vertex[0]);
                if (!list[k]._texCord.empty ()) {
                    glTexCoord2fv (list[k]._texCord[1]);
                }
                glNormal3fv (list[k]._normal[1]);
                glVertex3fv (list[k]._vertex[1]);
                if (!list[k]._texCord.empty ()) {
                    glTexCoord2fv (list[k]._texCord[2]);
                }
                glNormal3fv (list[k]._normal[2]);
                glVertex3fv (list[k]._vertex[2]);
                glEnd ();
            }
            if (k == list.size ()) {
                glDisable (GL_TEXTURE_2D);
            }
            else if (_model->_materials[list[k]._mat].hasTexture ()) {
                glDisable (GL_TEXTURE_2D);
            }
            j = k - 1;
            glPopMatrix ();
        }
        i = j - 1;

        glPopMatrix ();
    }
    glPopMatrix ();
}

void RenderModel3D::drawUsingSimple (const DrawableNode::RenderInfo& info, DrawType type,
                                     double alpha, bool disableMaterials) const
{
    glPushMatrix ();
    // Move the model
    DrawableUtil::multGLTransform (_model->getTransform ());

    if (!info._renderTransparent) {
        // Loop through the objects
        for (Model3D::Object3DGeneric::Ptr& objPtr : _model->getObjects ()) {
            if (const Model3D::Object3D< uint8_t >::Ptr objPtrT =
                    objPtr.cast< Model3D::Object3D< uint8_t > > ()) {
                drawUsingSimpleFct (info, *objPtrT, type, alpha, disableMaterials);
            }
            else if (const Model3D::Object3D< uint16_t >::Ptr objPtrT =
                         objPtr.cast< Model3D::Object3D< uint16_t > > ()) {
                drawUsingSimpleFct (info, *objPtrT, type, alpha, disableMaterials);
            }
            else if (const Model3D::Object3D< uint32_t >::Ptr objPtrT =
                         objPtr.cast< Model3D::Object3D< uint32_t > > ()) {
                drawUsingSimpleFct (info, *objPtrT, type, alpha, disableMaterials);
            }
            else {
                RW_THROW ("RenderModel3D could not recognize the type of Object3D in the model.");
            }
        }
    }
    else {
        std::vector< TPFace > list;

        for (Model3D::Object3DGeneric::Ptr& objPtr : _model->getObjects ()) {
            if (const Model3D::Object3D< uint8_t >::Ptr objPtrT =
                    objPtr.cast< Model3D::Object3D< uint8_t > > ()) {
                makeVertexList< uint8_t > (*objPtrT, list, cast< float > (_model->getTransform ()));
            }
            else if (const Model3D::Object3D< uint16_t >::Ptr objPtrT =
                         objPtr.cast< Model3D::Object3D< uint16_t > > ()) {
                makeVertexList< uint16_t > (
                    *objPtrT, list, cast< float > (_model->getTransform ()));
            }
            else if (const Model3D::Object3D< uint32_t >::Ptr objPtrT =
                         objPtr.cast< Model3D::Object3D< uint32_t > > ()) {
                makeVertexList< uint32_t > (
                    *objPtrT, list, cast< float > (_model->getTransform ()));
            }
            else {
                RW_THROW ("RenderModel3D could not recognize the type of Object3D in the model.");
            }
        }
        Transform3D< float > cTw = cast< float > (inverse (info._cam->getTransform ()));
        Transform3D< float > wTm = cast< float > (info._wTm);
        for (TPFace& f : list) {
            for (size_t i = 0; i < f._vertex.size (); i++) {
                Vector3D< float > v (f._vertex[i][0], f._vertex[i][1], f._vertex[i][2]);
                v = wTm * f._transform * v;
                v = cTw * v;
                f.dist2cam += v[2];
            }
        }
        std::sort (list.begin (), list.end ());
        drawTPFaceList (list, info, type, alpha, disableMaterials);
    }

    glPopMatrix ();
}

void RenderModel3D::drawUsingArrays (const DrawableNode::RenderInfo& info, DrawType type,
                                     double alpha) const
{
    glPushMatrix ();

    // Move the model
    DrawableUtil::multGLTransform (_model->getTransform ());
    // glScalef(scale, scale, scale);

    // Loop through the objects
    for (Model3D::Object3DGeneric::Ptr& objPtr : _model->getObjects ()) {
        if (const Model3D::Object3D< uint8_t >::Ptr objPtrT =
                objPtr.cast< Model3D::Object3D< uint8_t > > ()) {
            drawUsingArraysFct (info, *objPtrT, type, alpha);
        }
        else if (const Model3D::Object3D< uint16_t >::Ptr objPtrT =
                     objPtr.cast< Model3D::Object3D< uint16_t > > ()) {
            drawUsingArraysFct (info, *objPtrT, type, alpha);
        }
        else if (const Model3D::Object3D< uint32_t >::Ptr objPtrT =
                     objPtr.cast< Model3D::Object3D< uint32_t > > ()) {
            drawUsingArraysFct (info, *objPtrT, type, alpha);
        }
        else {
            RW_THROW ("RenderModel3D could not recognize the type of Object3D in the model.");
        }
    }

    glPopMatrix ();
}

template< class T >
void RenderModel3D::drawUsingSimpleFct (const DrawableNode::RenderInfo& info,
                                        const Model3D::Object3D< T >& obj,
                                        DrawableNode::DrawType type, double alpha,
                                        bool disableMaterials) const
{
    // for some reason opengl does not allow drawing to large meshes within glBegin/glEnd
    // so we split it up in smaller chunks
    glPushMatrix ();

    DrawableUtil::multGLTransform (obj._transform);
    if (obj._normals.size () != 0 && obj._vertices.size () != 0) {
        // Loop through the faces as sorted by material and draw them
        for (const typename Model3D::Object3DGeneric::MaterialMapData& data : obj._materialMap) {
            glPushMatrix ();
            if (!disableMaterials) {
                useMaterial (_model->_materials[data.matId], type, alpha);
            }

            if (_model->_materials[data.matId].hasTexture ()) {
                glEnable (GL_TEXTURE_2D);
                if (obj._mappedToFaces) {
                    glBegin (GL_TRIANGLES);
                    for (std::size_t i = data.startIdx; i < data.startIdx + data.size; i++) {
                        // draw faces
                        const rw::geometry::IndexedTriangle< T >& tri = obj._faces[i];
                        glTexCoord2fv (&obj._texCoords[i * 3](0));
                        glNormal3fv (&obj._normals[tri[0]](0));
                        glVertex3fv (&obj._vertices[tri[0]](0));

                        glTexCoord2fv (&obj._texCoords[i * 3 + 1](0));
                        glNormal3fv (&obj._normals[tri[1]](0));
                        glVertex3fv (&obj._vertices[tri[1]](0));

                        glTexCoord2fv (&obj._texCoords[i * 3 + 2](0));
                        glNormal3fv (&obj._normals[tri[2]](0));
                        glVertex3fv (&obj._vertices[tri[2]](0));
                    }
                    glEnd ();
                }
                else {
                    RW_ASSERT (obj._texCoords.size () == obj._normals.size ());
                    glBegin (GL_TRIANGLES);
                    for (std::size_t i = data.startIdx; i < data.startIdx + data.size; i++) {
                        // draw faces
                        const rw::geometry::IndexedTriangle< T >& tri = obj._faces[i];
                        glTexCoord2fv (&obj._texCoords[tri[0]](0));
                        glNormal3fv (&obj._normals[tri[0]](0));
                        glVertex3fv (&obj._vertices[tri[0]](0));

                        glTexCoord2fv (&obj._texCoords[tri[1]](0));
                        glNormal3fv (&obj._normals[tri[1]](0));
                        glVertex3fv (&obj._vertices[tri[1]](0));

                        glTexCoord2fv (&obj._texCoords[tri[2]](0));
                        glNormal3fv (&obj._normals[tri[2]](0));
                        glVertex3fv (&obj._vertices[tri[2]](0));
                    }
                    glEnd ();
                }
                glDisable (GL_TEXTURE_2D);
            }
            else {
                glBegin (GL_TRIANGLES);
                for (std::size_t i = data.startIdx; i < data.startIdx + data.size; i++) {
                    // draw faces
                    const rw::geometry::IndexedTriangle< T >& tri = obj._faces[i];
                    glNormal3fv (&obj._normals[tri[0]](0));
                    glVertex3fv (&obj._vertices[tri[0]](0));
                    glNormal3fv (&obj._normals[tri[1]](0));
                    glVertex3fv (&obj._vertices[tri[1]](0));
                    glNormal3fv (&obj._normals[tri[2]](0));
                    glVertex3fv (&obj._vertices[tri[2]](0));
                }
                glEnd ();
            }
            glPopMatrix ();
        }
    }

    // draw children
    // TODO: create method to switch normal rendering on and off
    for (const Model3D::Object3DGeneric::Ptr& child : obj._kids) {
        if (const Model3D::Object3D< uint8_t >::Ptr objPtrT =
                child.cast< Model3D::Object3D< uint8_t > > ()) {
            drawUsingSimpleFct< uint8_t > (info, *objPtrT, type, alpha, disableMaterials);
        }
        else if (const Model3D::Object3D< uint16_t >::Ptr objPtrT =
                     child.cast< Model3D::Object3D< uint16_t > > ()) {
            drawUsingSimpleFct< uint16_t > (info, *objPtrT, type, alpha, disableMaterials);
        }
        else if (const Model3D::Object3D< uint32_t >::Ptr objPtrT =
                     child.cast< Model3D::Object3D< uint32_t > > ()) {
            drawUsingSimpleFct< uint32_t > (info, *objPtrT, type, alpha, disableMaterials);
        }
        else {
            RW_THROW ("RenderModel3D could not recognize the type of Object3D in the model.");
        }
    }

    glPopMatrix ();
}

template< class T >
void RenderModel3D::drawUsingArraysFct (const DrawableNode::RenderInfo& info,
                                        const Model3D::Object3D< T >& obj,
                                        DrawableNode::DrawType type, double alpha) const
{
    glPushMatrix ();
    DrawableUtil::multGLTransform (obj._transform);

    if (obj._normals.size () != 0 && obj._vertices.size () != 0) {
        // Enable texture coordiantes, ngetormals, and vertices arrays
        if (obj.hasTexture ()) {
            glEnable (GL_TEXTURE_2D);
            glEnableClientState (GL_TEXTURE_COORD_ARRAY);
        }
        // if (lit)
        glEnableClientState (GL_NORMAL_ARRAY);

        glEnableClientState (GL_VERTEX_ARRAY);

        // Point them to the objects arrays
        if (obj.hasTexture ()) {
            glTexCoordPointer (2, GL_FLOAT, 0, &(obj._texCoords.at (0)[0]));
        }
        // if (lit) // if we use lightning... but we allways do
        glNormalPointer (GL_FLOAT, sizeof (Vector3D< float >), &(obj._normals.at (0)[0]));
        glVertexPointer (3, GL_FLOAT, sizeof (Vector3D< float >), &(obj._vertices.at (0)[0]));

        // Loop through the faces as sorted by material and draw them
        for (const typename Model3D::Object3DGeneric::MaterialMapData& data : obj._materialMap) {
            useMaterial (_model->_materials[data.matId], type, alpha);

            // Draw the faces using an index to the vertex array
            glDrawElements (GL_TRIANGLES,
                            GLsizei (data.size * 3),
                            GL_UNSIGNED_SHORT,
                            &(obj._faces.at (data.startIdx)));
        }

        if (obj.hasTexture ()) {
            glDisable (GL_TEXTURE_2D);
        }
    }
    // TODO: make sure polygons are also drawn
    // draw children
    for (const Model3D::Object3DGeneric::Ptr& child : obj._kids) {
        if (const Model3D::Object3D< uint8_t >::Ptr objPtrT =
                child.cast< Model3D::Object3D< uint8_t > > ()) {
            drawUsingArraysFct< uint8_t > (info, *objPtrT, type, alpha);
        }
        else if (const Model3D::Object3D< uint16_t >::Ptr objPtrT =
                     child.cast< Model3D::Object3D< uint16_t > > ()) {
            drawUsingArraysFct< uint16_t > (info, *objPtrT, type, alpha);
        }
        else if (const Model3D::Object3D< uint32_t >::Ptr objPtrT =
                     child.cast< Model3D::Object3D< uint32_t > > ()) {
            drawUsingArraysFct< uint32_t > (info, *objPtrT, type, alpha);
        }
        else {
            RW_THROW ("RenderModel3D could not recognize the type of Object3D in the model.");
        }
    }

    glPopMatrix ();

    // Show the normals?
    /*
    if (_shownormals)
    {
            // Loop through the vertices and normals and draw the normal
            for (int k = 0; k < Objects.at(i).numVerts * 3; k += 3)
            {
                    // Disable texturing
                    glDisable(GL_TEXTURE_2D);
                    // Disbale lighting if the model is lit
                    if (lit)
                            glDisable(GL_LIGHTING);
                    // Draw the normals blue
                    glColor3f(0.0f, 0.0f, 1.0f);

                    // Draw a line between the vertex and the end of the normal
                    glBegin(GL_LINES);
                    glVertex3f(
                            Objects.at(i).Vertexes.at(k),
                            Objects.at(i).Vertexes.at(k+1),
                            Objects.at(i).Vertexes.at(k+2));

                    glVertex3f(
                            Objects.at(i).Vertexes.at(k)+Objects.at(i).Normals.at(k),
                            Objects.at(i).Vertexes.at(k+1)+Objects.at(i).Normals.at(k+1),
                            Objects.at(i).Vertexes.at(k+2)+Objects.at(i).Normals.at(k+2));
                    glEnd();

                    // Reset the color to white
                    glColor3f(1.0f, 1.0f, 1.0f);
                    // If the model is lit then renable lighting
                    if (lit)
                            glEnable(GL_LIGHTING);
            }
    }
    */
}

void RenderModel3D::useMaterial (const Model3D::Material& mat, DrawType type, double alpha) const
{
    if (mat.hasTexture ()) {
        const RWGLTexture::Ptr tex = _textures[mat.getTextureID ()];
        const TextureData& tdata   = _model->getTextures<Model3DTextureType>()[mat.getTextureID ()];
        if (tdata.hasImageData ()) {
            tex->init (*tdata.getImageData ());
        }
        else {
            const Vector3D< float > rgb = tdata.getRGBData ();
            tex->init ((unsigned char) (255 * rgb[0]),
                       (unsigned char) (255 * rgb[1]),
                       (unsigned char) (255 * rgb[2]));
        }
        glBindTexture (GL_TEXTURE_2D, tex->getTextureID ());
    }

    if (mat.simplergb) {
        glColor4f (mat.rgb[0], mat.rgb[1], mat.rgb[2], (float) (mat.rgb[3] * alpha));
    }
    else {
        glMaterialfv (GL_FRONT, GL_SPECULAR, mat.specular);
        glMaterialfv (GL_FRONT, GL_SHININESS, &mat.shininess);
        glMaterialfv (GL_FRONT, GL_EMISSION, mat.emissive);
        //
        // ambient and defused are controlled using glcolor by enabling glColorMaterial
        // glMaterialfv(GL_FRONT, GL_AMBIENT, mat.ambient);
        // glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
        glColor4f (mat.rgb[0], mat.rgb[1], mat.rgb[2], (float) (mat.rgb[3] * alpha));
    }
}
