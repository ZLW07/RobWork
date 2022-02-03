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

#include "LoaderOBJ.hpp"

#include <rw/core/IOUtil.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/geometry/Triangulate.hpp>
#include <rw/loaders/ImageFactory.hpp>
#include <rw/math/EAA.hpp>

#include <fstream>

using namespace rw::loaders;
using namespace rw::graphics;
using namespace rw::graphics;
using namespace rw::math;
using namespace rw::core;
using namespace rw::geometry;

namespace rw { namespace loaders {

    /**
     * @brief Class for loading in OBJ files
     */
    class OBJReader
    {
      public:
        struct Face
        {
            int v;
            int vn;
            int vt;
        };
        struct MaterialFaces
        {
            // Index to our vertex array of all the faces that use this material
            std::vector< Face > subFaces;
            int MatIndex;    // An index to our materials
        };

        // The file can be made up of several objects
        struct Object
        {
            std::string name;                         // The object name
            std::vector< MaterialFaces > MatFaces;    // The faces are divided by materials
        };

      public:
        OBJReader ();
        ~OBJReader ();

        void Load (const std::string& fileName);
        void render (float alpha) const;
        void scale (float factor);
        void writeFile (const std::string& filename) const;
        void calcVertexNormals ();

        std::string _dirName;

        typedef std::pair< Vector2D< float >, float > imgCord;
        std::vector< Model3D::Material > _materials;    // The array of materials
        std::vector< TextureData > _textures;           // The array of Textures
        std::vector< Object > _objects;                 // The array of objects in the model
        std::vector< Vector3D< float > > _vertexes;     // The array of vertices
        std::vector< Vector3D< float > > _normals;      // The array of the normals for the vertices
        std::vector< imgCord > _texCoords;    // The array of texture coordinates for the vertices

      private:
        size_t _currentMat;

        char* rwStrtok (char* _Str, const char* _Delim, char** _Context)
        {
            int numDelim = static_cast< int > (strlen (_Delim));
            if (_Str)
                *_Context = _Str;

            if (!**_Context)
                return NULL;

            bool cont = true;
            while (cont) {
                for (int i = 0; i < numDelim; i++) {
                    if (**_Context == _Delim[i]) {
                        (*_Context)++;
                        cont = true;
                    }
                    else
                        cont = false;
                }
            }

            _Str = *_Context;

            while (**_Context) {
                for (int i = 0; i < numDelim; i++) {
                    if (**_Context == _Delim[i]) {
                        **_Context = 0;
                        (*_Context)++;
                        return _Str;
                    }
                }
                (*_Context)++;
            }

            if (!*_Str)
                return NULL;

            return _Str;
        }

      private:
        void parseMtlFile (const std::string& fileName);
        void writeMtlFile (const std::string& filename) const;

        typedef void (OBJReader::*FuncPtr) (char** next_token);
        std::map< std::string, FuncPtr > _objTypeMap;

        void parse_v (char** next_token);
        void parse_vt (char** next_token);
        void parse_vn (char** next_token);
        void parse_face (char** next_token);
        void parse_g (char** next_token);
        void parse_usemtl (char** next_token);
        void parse_mtllib (char** next_token);
        void parse_vp (char** next_token) { RW_WARN ("obj type 'vp' not implemented"); }
        void parse_cstype (char** next_token) { RW_WARN ("obj type 'cstype' not implemented"); }
        void parse_deg (char** next_token) { RW_WARN ("obj type 'deg' not implemented"); }
        void parse_bmat (char** next_token) { RW_WARN ("obj type 'bmat' not implemented"); }
        void parse_step (char** next_token) { RW_WARN ("obj type 'step' not implemented"); }
        void parse_p (char** next_token) { RW_WARN ("obj type 'p' not implemented"); }
        void parse_l (char** next_token) { RW_WARN ("obj type 'l' not implemented"); }
        void parse_curv (char** next_token) { RW_WARN ("obj type 'curv' not implemented"); }
        void parse_curv2 (char** next_token) { RW_WARN ("obj type 'curv2' not implemented"); }
        void parse_surf (char** next_token) { RW_WARN ("obj type 'surf' not implemented"); }
        void parse_parm (char** next_token) { RW_WARN ("obj type 'parm' not implemented"); }
        void parse_hole (char** next_token) { RW_WARN ("obj type 'hole' not implemented"); }
        void parse_scrv (char** next_token) { RW_WARN ("obj type 'scrv' not implemented"); }
        void parse_sp (char** next_token) { RW_WARN ("obj type 'sp' not implemented"); }
        void parse_end (char** next_token) { RW_WARN ("obj type 'end' not implemented"); }
        void parse_con (char** next_token) { RW_WARN ("obj type 'con' not implemented"); }
        void parse_mg (char** next_token) { RW_WARN ("obj type 'mg' not implemented"); }
        void parse_o (char** next_token) { RW_WARN ("obj type 'o' not implemented"); }
        void parse_bevel (char** next_token) { RW_WARN ("obj type 'bevel' not implemented"); }
        void parse_c_interp (char** next_token) { RW_WARN ("obj type 'c_interp' not implemented"); }
        void parse_d_interp (char** next_token) { RW_WARN ("obj type 'd_interp' not implemented"); }
        void parse_lod (char** next_token) { RW_WARN ("obj type 'lod' not implemented"); }
        void parse_shadow_obj (char** next_token)
        {
            RW_WARN ("obj type 'shadow_obj' not implemented");
        }
        void parse_trace_obj (char** next_token)
        {
            RW_WARN ("obj type 'trace_obj' not implemented");
        }
        void parse_ctech (char** next_token) { RW_WARN ("obj type 'ctech' not implemented"); }
        void parse_stech (char** next_token) { RW_WARN ("obj type 'stech' not implemented"); }
        void parse_s (char** next_token) { RW_WARN ("obj type 's' not implemented"); }

        typedef void (OBJReader::*MtlFuncPtr) (char** next_token);
        std::map< std::string, MtlFuncPtr > _mtlTypeMap;

        void parse_mtl_newmtl (char** next_token);
        void parse_mtl_illum (char** next_token);
        void parse_mtl_Kd (char** next_token);
        void parse_mtl_Ka (char** next_token);
        void parse_mtl_Ke (char** next_token);
        void parse_mtl_Tf (char** next_token);
        void parse_mtl_Ni (char** next_token);
        void parse_mtl_Ks (char** next_token);
        void parse_mtl_Ns (char** next_token);
        void parse_mtl_d (char** next_token);
        void parse_mtl_Tr (char** next_token);

        void parse_mtl_map_Ka (char** next_token);
        void parse_mtl_map_Ke (char** next_token);
        void parse_mtl_map_Kd (char** next_token);
        void parse_mtl_bump (char** next_token);
        void parse_mtl_map_bump (char** next_token);
        void parse_mtl_map_opacity (char** next_token);
        void parse_mtl_map_d (char** next_token);
        void parse_mtl_refl (char** next_token);
        void parse_mtl_map_kS (char** next_token);
        void parse_mtl_map_kA (char** next_token);
        void parse_mtl_map_Ns (char** next_token);

        int parseInt (char** next_token);
        float parseFloat (char** next_token);
        float parseOptionalFloat (char** next_token, float defaultVal);
    };

}}    // namespace rw::loaders

/*void OBJReader::Face::calcCommonNormal ()
{
    if (_element.size () >= 3) {
        Vector3D< float > p0 (_objReader->_geoVertices[_element[0]._v[0] - 1]._v[0],
                              _objReader->_geoVertices[_element[0]._v[0] - 1]._v[1],
                              _objReader->_geoVertices[_element[0]._v[0] - 1]._v[2]);
        Vector3D< float > p1 (_objReader->_geoVertices[_element[1]._v[0] - 1]._v[0],
                              _objReader->_geoVertices[_element[1]._v[0] - 1]._v[1],
                              _objReader->_geoVertices[_element[1]._v[0] - 1]._v[2]);
        Vector3D< float > p2 (_objReader->_geoVertices[_element[2]._v[0] - 1]._v[0],
                              _objReader->_geoVertices[_element[2]._v[0] - 1]._v[1],
                              _objReader->_geoVertices[_element[2]._v[0] - 1]._v[2]);
        Vector3D< float > p01 = p0 - p1;
        Vector3D< float > p21 = p2 - p1;
        _vCommonNorm          = cross (p21, p01);
        _vCommonNorm          = normalize (_vCommonNorm);
    }
}*/

OBJReader::OBJReader ()
{
    _objects.push_back (Object ());
    _objects.back ().name = "Default";

    _materials.push_back (Model3D::Material ("Default", 0.5, 0.5, 0.5));
    _currentMat = 0;

    _objTypeMap["v"]          = &OBJReader::parse_v;
    _objTypeMap["vt"]         = &OBJReader::parse_vt;
    _objTypeMap["vn"]         = &OBJReader::parse_vn;
    _objTypeMap["f"]          = &OBJReader::parse_face;
    _objTypeMap["g"]          = &OBJReader::parse_g;
    _objTypeMap["o"]          = &OBJReader::parse_g;    // TODO Should o be parsed differently
    _objTypeMap["usemtl"]     = &OBJReader::parse_usemtl;
    _objTypeMap["mtllib"]     = &OBJReader::parse_mtllib;
    _objTypeMap["vp"]         = &OBJReader::parse_vp;
    _objTypeMap["cstype"]     = &OBJReader::parse_cstype;
    _objTypeMap["deg"]        = &OBJReader::parse_deg;
    _objTypeMap["bmat"]       = &OBJReader::parse_bmat;
    _objTypeMap["step"]       = &OBJReader::parse_step;
    _objTypeMap["p"]          = &OBJReader::parse_p;
    _objTypeMap["l"]          = &OBJReader::parse_l;
    _objTypeMap["curv"]       = &OBJReader::parse_curv;
    _objTypeMap["curv2"]      = &OBJReader::parse_curv2;
    _objTypeMap["surf"]       = &OBJReader::parse_surf;
    _objTypeMap["parm"]       = &OBJReader::parse_parm;
    _objTypeMap["hole"]       = &OBJReader::parse_hole;
    _objTypeMap["scrv"]       = &OBJReader::parse_scrv;
    _objTypeMap["sp"]         = &OBJReader::parse_sp;
    _objTypeMap["end"]        = &OBJReader::parse_end;
    _objTypeMap["con"]        = &OBJReader::parse_con;
    _objTypeMap["mg"]         = &OBJReader::parse_mg;
    _objTypeMap["o"]          = &OBJReader::parse_o;
    _objTypeMap["bevel"]      = &OBJReader::parse_bevel;
    _objTypeMap["c_interp"]   = &OBJReader::parse_c_interp;
    _objTypeMap["d_interp"]   = &OBJReader::parse_d_interp;
    _objTypeMap["lod"]        = &OBJReader::parse_lod;
    _objTypeMap["shadow_obj"] = &OBJReader::parse_shadow_obj;
    _objTypeMap["trace_obj"]  = &OBJReader::parse_trace_obj;
    _objTypeMap["ctech"]      = &OBJReader::parse_ctech;
    _objTypeMap["stech"]      = &OBJReader::parse_stech;
    _objTypeMap["s"]          = &OBJReader::parse_s;

    _mtlTypeMap["newmtl"]      = &OBJReader::parse_mtl_newmtl;
    _mtlTypeMap["illum"]       = &OBJReader::parse_mtl_illum;
    _mtlTypeMap["Kd"]          = &OBJReader::parse_mtl_Kd;
    _mtlTypeMap["Ka"]          = &OBJReader::parse_mtl_Ka;
    _mtlTypeMap["Ke"]          = &OBJReader::parse_mtl_Ke;
    _mtlTypeMap["Tf"]          = &OBJReader::parse_mtl_Tf;
    _mtlTypeMap["Ni"]          = &OBJReader::parse_mtl_Ni;
    _mtlTypeMap["Ks"]          = &OBJReader::parse_mtl_Ks;
    _mtlTypeMap["Ns"]          = &OBJReader::parse_mtl_Ns;
    _mtlTypeMap["d"]           = &OBJReader::parse_mtl_d;
    _mtlTypeMap["Tr"]          = &OBJReader::parse_mtl_Tr;
    _mtlTypeMap["map_Kd"]      = &OBJReader::parse_mtl_map_Kd;
    _mtlTypeMap["map_Ka"]      = &OBJReader::parse_mtl_map_Ka;
    _mtlTypeMap["map_bump"]    = &OBJReader::parse_mtl_map_bump;
    _mtlTypeMap["bump"]        = &OBJReader::parse_mtl_map_bump;
    _mtlTypeMap["map_opacity"] = &OBJReader::parse_mtl_map_opacity;
    _mtlTypeMap["map_d"]       = &OBJReader::parse_mtl_map_d;
    _mtlTypeMap["refl"]        = &OBJReader::parse_mtl_refl;
    _mtlTypeMap["map_kS"]      = &OBJReader::parse_mtl_map_kS;
    _mtlTypeMap["map_kA"]      = &OBJReader::parse_mtl_map_kA;
    _mtlTypeMap["map_Ns"]      = &OBJReader::parse_mtl_map_Ns;
}

OBJReader::~OBJReader ()
{}

void OBJReader::parse_v (char** next_token)
{
    float x = parseFloat (next_token);
    float y = parseFloat (next_token);
    float z = parseFloat (next_token);
    _vertexes.push_back ({x, y, z});
}

void OBJReader::parse_vt (char** next_token)
{
    float u = parseFloat (next_token);
    float v = parseOptionalFloat (next_token, 0.0);
    float w = parseOptionalFloat (next_token, 0.0);
    _texCoords.push_back (imgCord ({u, v}, w));
}

void OBJReader::parse_vn (char** next_token)
{
    float i = parseFloat (next_token);
    float j = parseFloat (next_token);
    float k = parseFloat (next_token);
    _normals.push_back ({i, j, k});
}

void OBJReader::parse_g (char** next_token)
{
    _objects.push_back (Object ());
    if (strlen (*next_token) != 0) {
        _objects.back ().name = *next_token;
    }
    else {
        _objects.back ().name = "Default" + std::to_string (_objects.size ());
    }
}

void OBJReader::parse_face (char** next_token)
{
    MaterialFaces face;

    char* token;
    while ((token = rwStrtok (NULL, " \t", next_token)) != NULL) {
        char* p = token;
        bool cont;
        int v, vt = -1, vn = -1;

        // vertex index
        while (*p != 0 && *p != '/')
            p++;
        cont  = *p != 0;
        *p    = 0;
        v     = atoi (token);
        token = ++p;

        // texture vertex index (optional)
        if (cont) {
            while (*p != 0 && *p != '/')
                p++;
            cont  = *p != 0;
            *p    = 0;
            vt    = token != p ? atoi (token) : -1;
            token = ++p;

            // vertex normal index (optional)
            if (cont) {
                while (*p != 0)
                    p++;
                *p    = 0;
                vn    = token != p ? atoi (token) : -1;
                token = ++p;
            }
        }
        face.subFaces.push_back ({v - 1, vn - 1, vt - 1});
    }
    face.MatIndex = int (_currentMat);
    _objects.back ().MatFaces.push_back (face);
}

void OBJReader::parse_usemtl (char** next_token)
{
    char* token   = rwStrtok (NULL, "\r\n", next_token);
    bool foundMat = false;
    for (size_t i = 0; i < _materials.size (); i++) {
        if (_materials[i].name == std::string (token)) {
            foundMat    = true;
            _currentMat = i;
        }
    }
    if (!foundMat) {
        RW_THROW ("Material '" << token << "' not defined");
    }
}

void OBJReader::parse_mtllib (char** next_token)
{
    char* token;
    while ((token = rwStrtok (NULL, " \t", next_token)) != NULL) {
        parseMtlFile (_dirName + token);
    }
}

void OBJReader::parse_mtl_newmtl (char** next_token)
{
    char* token = rwStrtok (NULL, "\r\n", next_token);
    _materials.push_back (Model3D::Material (token, 0.5, 0.5, 0.5));
}

void OBJReader::parse_mtl_illum (char** next_token)
{}

void OBJReader::parse_mtl_Kd (char** next_token)
{
    _materials.back ().rgb[0] = parseFloat (next_token);
    _materials.back ().rgb[1] = parseFloat (next_token);
    _materials.back ().rgb[2] = parseFloat (next_token);
}

void OBJReader::parse_mtl_Ka (char** next_token)
{
    _materials.back ().ambient[0] = parseFloat (next_token);
    _materials.back ().ambient[1] = parseFloat (next_token);
    _materials.back ().ambient[2] = parseFloat (next_token);
    _materials.back ().simplergb  = false;
}

void OBJReader::parse_mtl_Ke (char** next_token)
{
    _materials.back ().emissive[0] = parseFloat (next_token);
    _materials.back ().emissive[1] = parseFloat (next_token);
    _materials.back ().emissive[2] = parseFloat (next_token);
    _materials.back ().simplergb   = false;
}

void OBJReader::parse_mtl_Tf (char** next_token)
{
    // TODO this specifies how much color is let trough in RGB "Transmission Filter"
}

void OBJReader::parse_mtl_Ni (char** next_token)
{
    // TODO
    RW_WARN ("mlt type 'Ni' not implemented");
}

void OBJReader::parse_mtl_Ks (char** next_token)
{
    _materials.back ().specular[0] = parseFloat (next_token);
    _materials.back ().specular[1] = parseFloat (next_token);
    _materials.back ().specular[2] = parseFloat (next_token);
}

void OBJReader::parse_mtl_Ns (char** next_token)
{
    _materials.back ().shininess = parseFloat (next_token) / 1000.0f * 128.0f;
}

void OBJReader::parse_mtl_d (char** next_token)
{
    _materials.back ().transparency = parseFloat (next_token);
}

void OBJReader::parse_mtl_Tr (char** next_token)
{
    _materials.back ().transparency = 1.0f - parseFloat (next_token);
}

void OBJReader::parse_mtl_map_Ka (char** next_token)
{
    // TODO: implements map_Ka functionality
}

void OBJReader::parse_mtl_map_Kd (char** next_token)
{
    char* token;
    while (strlen (*next_token) != 0) {
        token = rwStrtok (NULL, " \t", next_token);
        if (token[0] == '-') {
            // TODO: handle options
        }
        else {
            _textures.push_back (
                TextureData (token, ImageLoader::Factory::load (_dirName + token)));
            _materials.back ().texId = short (_textures.size () - 1);
        }
    }
}

void OBJReader::parse_mtl_map_opacity (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'map_opacity' not implemented");
    }
}

void OBJReader::parse_mtl_map_bump (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'map_bump' not implemented");
    }
}

void OBJReader::parse_mtl_map_d (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'map_d' not implemented");
    }
}

void OBJReader::parse_mtl_refl (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'refl' not implemented");
    }
}

void OBJReader::parse_mtl_map_kS (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'map_kS' not implemented");
    }
}

void OBJReader::parse_mtl_map_kA (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'map_kA' not implemented");
    }
}

void OBJReader::parse_mtl_map_Ns (char** next_token)
{
    if (strlen (*next_token) != 0) {
        RW_WARN ("mlt type 'map_Ns' not implemented");
    }
}

int OBJReader::parseInt (char** next_token)
{
    char* token = rwStrtok (NULL, " \t", next_token);
    if (token == NULL)
        RW_THROW ("Parse error");
    return atoi (token);
}

float OBJReader::parseFloat (char** next_token)
{
    char* token = rwStrtok (NULL, " \t", next_token);
    if (token == NULL)
        RW_THROW ("Parse error");
    return static_cast< float > (atof (token));
}

float OBJReader::parseOptionalFloat (char** next_token, float defaultVal)
{
    char* token = rwStrtok (NULL, " \t", next_token);
    if (token == NULL)
        return defaultVal;
    else
        return static_cast< float > (atof (token));
}

void OBJReader::Load (const std::string& fileName)
{
    int lineNum = 1;
    char* buffer;
    char* line;
    char* next_line = 0;

    std::vector< char > vecBuf;
    IOUtil::readFile (fileName, vecBuf);
    vecBuf.push_back (0);
    buffer   = &vecBuf[0];
    _dirName = rw::core::StringUtil::getDirectoryName (fileName);

    // Read first line
    line = rwStrtok (buffer, "\r\n", &next_line);
    while (line != NULL) {
        if (line[0] != '#') {
            char* token;
            char* next_token;
            token                                         = rwStrtok (line, " \t", &next_token);
            std::map< std::string, FuncPtr >::iterator it = _objTypeMap.find (token);
            if (it == _objTypeMap.end ())
                RW_WARN ("Unknown type: '" << token << "' in line " << lineNum);
            (this->*it->second) (&next_token);
        }
        line = rwStrtok (NULL, "\r\n", &next_line);
        lineNum++;
    }
}

void OBJReader::parseMtlFile (const std::string& fileName)
{
    int lineNum = 1;
    char* buffer;
    char* line;
    char* next_line = 0;

    std::vector< char > vecBuf;
    IOUtil::readFile (fileName, vecBuf);
    vecBuf.push_back (0);
    buffer = &vecBuf[0];

    // Read first line
    line = rwStrtok (buffer, "\r\n", &next_line);

    while (line != NULL) {
        if (line[0] != '#') {
            char* token;
            char* next_token;
            token                                            = rwStrtok (line, " \t", &next_token);
            std::map< std::string, MtlFuncPtr >::iterator it = _mtlTypeMap.find (token);
            if (it == _mtlTypeMap.end ())
                RW_THROW ("Unknown mtl type: '" << token << "' in line " << lineNum);
            (this->*it->second) (&next_token);
        }
        line = rwStrtok (NULL, "\r\n", &next_line);
        lineNum++;
    }
}

void OBJReader::scale (float factor)
{
    for (Vector3D< float >& v : this->_vertexes) {
        v *= factor;
    }
}

void OBJReader::writeFile (const std::string& filename) const
{
    std::string objFilename = filename + ".obj";
    std::string mtlFilename = filename + ".mtl";

    writeMtlFile (mtlFilename);

    std::ofstream out (objFilename.c_str ());
    if (!out)
        RW_THROW ("Unable to write '" << objFilename << "'");

    out << "mtllib " << mtlFilename.substr (mtlFilename.rfind ("\\") + 1) << std::endl;

    /*std::vector< Vec3 >::const_iterator it;
    out.precision (6);
    out.setf (std::ios_base::fixed);
    for (it = _geoVertices.begin (); it != _geoVertices.end (); it++)
        out << "v " << it->_v[0] << " " << it->_v[1] << " " << it->_v[2] << std::endl;

    for (it = _textVertices.begin (); it != _textVertices.end (); it++)
        out << "vt " << it->_v[0] << " " << it->_v[1] << " " << it->_v[2] << std::endl;

    for (it = _vertexNormals.begin (); it != _vertexNormals.end (); it++)
        out << "vn " << it->_v[0] << " " << it->_v[1] << " " << it->_v[2] << std::endl;

    std::vector< RenderItem* >::const_iterator it2;
    for (it2 = _renderItems.begin (); it2 != _renderItems.end (); it2++)
        (*it2)->write (out);
        */
    out.close ();
}

void OBJReader::writeMtlFile (const std::string& filename) const
{
    std::ofstream out (filename.c_str ());
    if (!out)
        RW_THROW ("Unable to write '" << filename << "'");
    out.precision (6);
    out.setf (std::ios_base::fixed);

    /*std::map< std::string, Mtl* >::const_iterator it;
    for (it = _materials.begin (); it != _materials.end (); it++)
        it->second->Write (out);
        */
    out.close ();
}

void OBJReader::calcVertexNormals ()
{
    /*std::map< int, std::vector< Face* > > geoVertexFaceMap;
    std::vector< RenderItem* >::iterator rit;
    std::vector< IVec3 >::iterator eit;
    for (rit = _renderItems.begin (); rit != _renderItems.end (); rit++) {
        if (typeid (**rit) == typeid (class OBJReader::Face)) {
            Face* face = static_cast< Face* > (*rit);
            for (eit = face->_element.begin (); eit != face->_element.end (); eit++) {
                geoVertexFaceMap[eit->_v[0]].push_back (face);
                eit->_v[2] = eit->_v[0];
            }
            face->calcCommonNormal ();
        }
    }

    std::map< int, std::vector< Face* > >::iterator git;
    for (git = geoVertexFaceMap.begin (); git != geoVertexFaceMap.end (); git++) {
        Vector3D< float > tmp (0.0f, 0.0f, 0.0f);
        std::vector< Face* >::iterator vit;
        for (vit = git->second.begin (); vit != git->second.end (); vit++)
            tmp += (*vit)->_vCommonNorm;
        tmp /= static_cast< float > (git->second.size ());
        _vertexNormals.push_back (Vec3 (tmp[0], tmp[1], tmp[2]));
    }*/
}

namespace {
template< class T >
void triangulatePolygon (IndexedPolygonN< T >& poly,
                         std::vector< rw::math::Vector3D< float > >& verts,
                         std::vector< IndexedTriangle< T > >& res)
{
    // calculate poly normal from first three vertices
    Vector3D<> v0 = cast< double > (verts[poly[0]]);
    Vector3D<> v1 = cast< double > (verts[poly[1]]);
    Vector3D<> v2 = cast< double > (verts[poly[2]]);
    Vector3D<> n  = normalize (cross (v1 - v0, v2 - v0));
    // std::cout << "-" << v0 << "\n-" << v1 << "\n-" << v2 << "\n-" << n << std::endl;

    EAA<> eaa (n, Vector3D<> (0, 0, 1));
    Rotation3D<> rotNtoZ = eaa.toRotation3D ();
    // make vector of 2d points
    std::vector< Vector2D<> > points (poly.size ());
    for (size_t j = 0; j < poly.size (); j++) {
        // rotate each point such that the xy-plane is perpendicular to the normal
        Vector3D<> v = rotNtoZ * cast< double > (verts[poly[j]]);
        // std::cout << v << std::endl;
        points[j](0) = v (0);
        points[j](1) = v (1);
    }

    // now do the triangulation
    std::vector< int > indices;
    int iidx = 0;
    if (Triangulate::processPoints (points, indices)) {
        while (iidx < (int) indices.size ()) {
            IndexedTriangle< T > tri (
                poly[indices[iidx]], poly[indices[iidx + 1]], poly[indices[iidx + 2]]);
            res.push_back (tri);
            iidx += 3;
        }
    }
    else {
        RW_WARN ("Could not triangulate polygon face. Check face for overlapping points!");
    }
}
}    // namespace

Model3D::Ptr LoaderOBJ::load (const std::string& name)
{
    // Start by storing the current locale. This is retrieved by passing NULL to setlocale
    std::string locale = setlocale (LC_ALL, NULL);
    // We set the locale to make sure things are parsed correctly in from file
    setlocale (LC_ALL, "C");
    OBJReader reader;
    reader.Load (name);
    // Restore the old locale
    setlocale (LC_ALL, locale.c_str ());

    Model3D::Ptr model (ownedPtr (new Model3D (name)));
    model->getTextures<Model3DTextureType>() = reader._textures;
    model->_materials = reader._materials;

    Model3D::Object3DGeneric::Ptr obj;
    Model3D::Object3D< uint8_t >::Ptr obj8;
    Model3D::Object3D< uint16_t >::Ptr obj16;
    Model3D::Object3D< uint32_t >::Ptr obj32;

    model->_objects.resize (reader._objects.size ());
    std::vector< Model3D::Object3DGeneric::Ptr >& objects = model->_objects;

    for (size_t i = 0; i < objects.size (); i++) {
        OBJReader::Object& obj_src = reader._objects[i];

        // First count the vertices
        std::size_t vertexPoints = 0;
        for (OBJReader::MaterialFaces& matFace : obj_src.MatFaces) {
            vertexPoints += matFace.subFaces.size ();
        }

        if (vertexPoints < UINT8_MAX) {
            obj = obj8 = ownedPtr (new Model3D::Object3D< uint8_t > (obj_src.name));
        }
        else if (vertexPoints < UINT16_MAX) {
            obj = obj16 = ownedPtr (new Model3D::Object3D< uint16_t > (obj_src.name));
        }
        else if (vertexPoints < UINT32_MAX) {
            obj = obj32 = ownedPtr (new Model3D::Object3D< uint32_t > (obj_src.name));
        }
        else {
            RW_THROW ("LoaderOBJ can not load file " << name << " as it has too many vertices ("
                                                     << vertexPoints << " with max of "
                                                     << UINT32_MAX << ")!");
        }

        // copy MatFaces
        for (size_t j = 0; j < obj_src.MatFaces.size (); j++) {
            const OBJReader::MaterialFaces& matFace = obj_src.MatFaces[j];

            if (model->_materials[matFace.MatIndex].hasTexture ())
                obj->_hasTexture = true;

            obj->setMaterial (matFace.MatIndex);

            int index_s = int (obj->_vertices.size ());
            for (const OBJReader::Face& subFace : matFace.subFaces) {
                obj->_vertices.push_back (reader._vertexes[subFace.v]);
                if (0 <= subFace.vn) {
                    obj->_normals.push_back (reader._normals[subFace.vn]);
                }
                if (0 <= subFace.vt) {
                    obj->_texCoords.push_back (reader._texCoords[subFace.vt].first);
                }
            }

            if (matFace.subFaces.size () < 3) {
                RW_WARN ("An OBJ surface with only 2 vertices detected! It will be ignored!");
            }
            else if (matFace.subFaces.size () == 3) {
                // use TriangleUtil toIndexedTriMesh, though remember the normals
                if (vertexPoints < UINT8_MAX) {
                    obj8->addTriangle (rw::geometry::IndexedTriangle< uint8_t > (
                        static_cast< uint8_t > (index_s + 0),
                        static_cast< uint8_t > (index_s + 1),
                        static_cast< uint8_t > (index_s + 2)));
                }
                else if (vertexPoints < UINT16_MAX) {
                    obj16->addTriangle (rw::geometry::IndexedTriangle< uint16_t > (
                        static_cast< uint16_t > (index_s + 0),
                        static_cast< uint16_t > (index_s + 1),
                        static_cast< uint16_t > (index_s + 2)));
                }
                else if (vertexPoints < UINT32_MAX) {
                    obj32->addTriangle (rw::geometry::IndexedTriangle< uint32_t > (
                        static_cast< uint32_t > (index_s + 0),
                        static_cast< uint32_t > (index_s + 1),
                        static_cast< uint32_t > (index_s + 2)));
                }
            }
            else {
                // its a polygon, since we don't support that in Model3D, we make triangles of
                // it
                if (vertexPoints < UINT8_MAX) {
                    IndexedPolygonN< uint8_t > poly (matFace.subFaces.size ());

                    for (size_t k = 0; k < matFace.subFaces.size (); k++) {
                        poly[k] = (uint8_t) index_s + uint8_t (k);
                    }

                    std::vector< IndexedTriangle< uint8_t > > tris;
                    triangulatePolygon (poly, obj8->_vertices, tris);
                    obj8->addTriangles (tris);
                }
                else if (vertexPoints < UINT16_MAX) {
                    IndexedPolygonN< uint16_t > poly (matFace.subFaces.size ());

                    for (size_t k = 0; k < matFace.subFaces.size (); k++) {
                        poly[k] = (uint16_t) index_s + uint16_t (k);
                    }

                    std::vector< IndexedTriangle< uint16_t > > tris;
                    triangulatePolygon (poly, obj16->_vertices, tris);
                    obj16->addTriangles (tris);
                }
                else if (vertexPoints < UINT32_MAX) {
                    IndexedPolygonN< uint32_t > poly (matFace.subFaces.size ());

                    for (size_t k = 0; k < matFace.subFaces.size (); k++) {
                        poly[k] = (uint32_t) index_s + uint32_t (k);
                    }

                    std::vector< IndexedTriangle< uint32_t > > tris;
                    triangulatePolygon (poly, obj32->_vertices, tris);
                    obj32->addTriangles (tris);
                }
            }
        }

        obj->_mappedToFaces = false;
        model->_objects[i]  = obj;
    }
    return model;
}
