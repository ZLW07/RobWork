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



#include <gtest/gtest.h>

#include <rw/loaders/Model3DFactory.hpp>

#include <string>

#include "../TestEnvironment.hpp"

using rw::graphics::Model3D;
using rw::loaders::Model3DFactory;

TEST(GraphicsTest, testSTLLoading ){
    // test loading stl file
    Model3D::Ptr stlaObject =
    		Model3DFactory::loadModel( TestEnvironment::testfilesDir() + "geoms/chair.stla", "chair" );
    Model3D::Ptr stlbObject =
    		Model3DFactory::loadModel( TestEnvironment::testfilesDir() + "geoms/cube.stlb", "cube" );
}

TEST(GraphicsTest, testAC3DLoading ){
    // test loading AC3D file
    Model3D::Ptr ac3dObject =
            Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/Gantry0.ac", "gantry");
    Model3D::Ptr ac3dObject1 =
            Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/Gantry0.ac3d", "gantry1");
}

TEST(GraphicsTest, testOBJLoading ){
    // test loading OBJ file
    Model3D::Ptr objObject =
            Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/fod1.obj", "fod1");
}

TEST(GraphicsTest, testTRILoading ){
    // test loading TRI file
    Model3D::Ptr objObject =
            Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/Rob-0.tri", "rob-0");
}

TEST(GraphicsTest, test3DSLoading ){
    // test loading 3ds file
    Model3D::Ptr objObject =
            Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/motor.3ds", "motor");
}

/*
BOOST_AUTO_TEST_CASE( testIVGLoading ){
    // test loading 3ds file
    rwlibs::opengl::Drawable::Ptr objObject =
            DrawableFactory::loadDrawableFile(testFilePath() + "geoms/staubli0.ivg", "staubli");
}
*/

TEST(GraphicsTest, testDrawableFactory)
{
    // test ascii stl format load
    Model3D::Ptr stlaObject = Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/chair", "chair3");
    Model3D::Ptr stlbObject = Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/cube", "cube3");
    Model3D::Ptr ac3dObject = Model3DFactory::loadModel(TestEnvironment::testfilesDir() + "geoms/Environment","environment");

}
