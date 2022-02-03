/******************************************************************************
 * Copyright 2020 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef ROBWORK_GTEST_TASK_TASK_XML_HPP_
#define ROBWORK_GTEST_TASK_TASK_XML_HPP_

#include <string>
#include <ostream>

static const std::string& getTaskXML() {
    static std::string xml;
    if (!xml.empty())
        return xml;
    std::stringstream stream;
    // Generate with: cat file.xml | sed 's/\"/\\\"/g' | awk '{ print "stream << \""$0 "\" << std::endl;" }'
    stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
    stream << "<QTask>" << std::endl;
    stream << " <Targets>" << std::endl;
    stream << "  <QTarget id=\"0\">" << std::endl;
    stream << "   <Q>1</Q>" << std::endl;
    stream << "   <Id/>" << std::endl;
    stream << "   <Index>0</Index>" << std::endl;
    stream << "   <PropertyMap/>" << std::endl;
    stream << "  </QTarget>" << std::endl;
    stream << "  <QTarget id=\"1\">" << std::endl;
    stream << "   <Q>2</Q>" << std::endl;
    stream << "   <Id/>" << std::endl;
    stream << "   <Index>1</Index>" << std::endl;
    stream << "   <PropertyMap/>" << std::endl;
    stream << "  </QTarget>" << std::endl;
    stream << "  <QTarget id=\"2\">" << std::endl;
    stream << "   <Q>3</Q>" << std::endl;
    stream << "   <Id/>" << std::endl;
    stream << "   <Index>2</Index>" << std::endl;
    stream << "   <PropertyMap/>" << std::endl;
    stream << "  </QTarget>" << std::endl;
    stream << " </Targets>" << std::endl;
    stream << " <Entities>" << std::endl;
    stream << "  <Motion type=\"P2P\">" << std::endl;
    stream << "   <Start>0</Start>" << std::endl;
    stream << "   <End>1</End>" << std::endl;
    stream << "   <Id/>" << std::endl;
    stream << "   <Index>3</Index>" << std::endl;
    stream << "   <PropertyMap/>" << std::endl;
    stream << "  </Motion>" << std::endl;
    stream << "  <Motion type=\"Linear\">" << std::endl;
    stream << "   <Start>1</Start>" << std::endl;
    stream << "   <End>2</End>" << std::endl;
    stream << "   <Id/>" << std::endl;
    stream << "   <Index>4</Index>" << std::endl;
    stream << "   <PropertyMap/>" << std::endl;
    stream << "  </Motion>" << std::endl;
    stream << "  <Action type=\"1\">" << std::endl;
    stream << "   <Id/>" << std::endl;
    stream << "   <Index>5</Index>" << std::endl;
    stream << "   <PropertyMap/>" << std::endl;
    stream << "  </Action>" << std::endl;
    stream << " </Entities>" << std::endl;
    stream << " <Id/>" << std::endl;
    stream << " <Index>-1</Index>" << std::endl;
    stream << " <PropertyMap/>" << std::endl;
    stream << "</QTask>" << std::endl;
    xml = stream.str();
    return xml;
}

static const std::string& getTaskXMLXerces() {
    static std::string xml;
    if (!xml.empty())
        return xml;
    std::stringstream stream;
    // Generate with: cat file.xml | sed 's/\"/\\\"/g' | awk '{ print "stream << \""$0 "\" << std::endl;" }'
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>" << std::endl;
    stream << "<QTask>" << std::endl;
    stream << "" << std::endl;
    stream << "  <Targets>" << std::endl;
    stream << "    <QTarget id=\"0\">" << std::endl;
    stream << "      <Q>1</Q>" << std::endl;
    stream << "      <Id></Id>" << std::endl;
    stream << "      <Index>0</Index>" << std::endl;
    stream << "      <PropertyMap/>" << std::endl;
    stream << "    </QTarget>" << std::endl;
    stream << "    <QTarget id=\"1\">" << std::endl;
    stream << "      <Q>2</Q>" << std::endl;
    stream << "      <Id></Id>" << std::endl;
    stream << "      <Index>1</Index>" << std::endl;
    stream << "      <PropertyMap/>" << std::endl;
    stream << "    </QTarget>" << std::endl;
    stream << "    <QTarget id=\"2\">" << std::endl;
    stream << "      <Q>3</Q>" << std::endl;
    stream << "      <Id></Id>" << std::endl;
    stream << "      <Index>2</Index>" << std::endl;
    stream << "      <PropertyMap/>" << std::endl;
    stream << "    </QTarget>" << std::endl;
    stream << "  </Targets>" << std::endl;
    stream << "" << std::endl;
    stream << "  <Entities>" << std::endl;
    stream << "    <Motion type=\"P2P\">" << std::endl;
    stream << "      <Start>0</Start>" << std::endl;
    stream << "      <End>1</End>" << std::endl;
    stream << "      <Id></Id>" << std::endl;
    stream << "      <Index>3</Index>" << std::endl;
    stream << "      <PropertyMap/>" << std::endl;
    stream << "    </Motion>" << std::endl;
    stream << "    <Motion type=\"Linear\">" << std::endl;
    stream << "      <Start>1</Start>" << std::endl;
    stream << "      <End>2</End>" << std::endl;
    stream << "      <Id></Id>" << std::endl;
    stream << "      <Index>4</Index>" << std::endl;
    stream << "      <PropertyMap/>" << std::endl;
    stream << "    </Motion>" << std::endl;
    stream << "    <Action type=\"1\">" << std::endl;
    stream << "      <Id></Id>" << std::endl;
    stream << "      <Index>5</Index>" << std::endl;
    stream << "      <PropertyMap/>" << std::endl;
    stream << "    </Action>" << std::endl;
    stream << "  </Entities>" << std::endl;
    stream << "" << std::endl;
    stream << "  <Id></Id>" << std::endl;
    stream << "" << std::endl;
    stream << "  <Index>-1</Index>" << std::endl;
    stream << "" << std::endl;
    stream << "  <PropertyMap/>" << std::endl;
    stream << "" << std::endl;
    stream << "</QTask>" << std::endl;
    xml = stream.str();
    return xml;
}

#endif /* ROBWORK_GTEST_TASK_TASK_XML_HPP_ */
