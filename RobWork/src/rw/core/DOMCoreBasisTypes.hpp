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

#ifndef RW_CORE_BASISTYPES_HPP
#define RW_CORE_BASISTYPES_HPP

#if !defined(SWIG)
#include <rw/core/DOMElem.hpp>

#include <string>
#endif
namespace rw { namespace core {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Utility class to help read in the content of a XML-files parsed with Xerces
     * The Class is part of the core Library in a limited edition. Use DOMBasisTypes for a full
     * implementation
     */
    class DOMCoreBasisTypes
    {
      public:
        /**
         * @brief Identifier for rw::math::Q in the XML format.
         * @return the identifier.
         */
        static const std::string& idQ ();
        /**
         * @brief Identifier for rw::math::Vector3D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idVector3D ();
        /**
         * @brief Identifier for rw::math::Vector2D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idVector2D ();
        /**
         * @brief Identifier for rw::math::Rotation3D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idRotation3D ();
        /**
         * @brief Identifier for rw::math::RPY<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idRPY ();
        /**
         * @brief Identifier for rw::math::EAA<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idEAA ();
        /**
         * @brief Identifier for rw::math::Quaternion<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idQuaternion ();
        /**
         * @brief Identifier for rw::math::Rotation2D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idRotation2D ();
        /**
         * @brief Identifier for single angle used to specify rw::math::Rotation2D<> in the XML
         * format.
         * @return the identifier.
         */
        static const std::string& idRotation2DAngle ();
        /**
         * @brief Identifier for rw::math::Transform2D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idTransform2D ();
        /**
         * @brief Identifier for rw::math::Transform3D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idTransform3D ();
        /**
         * @brief Identifier for rw::math::MatrixXd<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idMatrix ();
        /**
         * @brief Identifier for rw::math::VelocityScrew6D<> in the XML format.
         * @return the identifier.
         */
        static const std::string& idVelocityScrew6D ();
        /**
         * @brief Identifier for the position specification used in Transform3D.
         * @return the identifier.
         */
        static const std::string& idPos ();

        /**
         * @brief Identifier for specifying the linear part in a VelocityScrew6D.
         * @return the identifier.
         */
        static const std::string& idLinear ();

        /**
         * @brief Identifier for specifying the angular part in a VelocityScrew6D.
         * @return the identifier.
         */
        static const std::string& idAngular ();

        /**
         * @brief Identifier for specifying a State.
         * @return the identifier.
         */
        static const std::string& idState ();

        /**
         * @brief Identifier for specifying a State.
         * @return the identifier.
         */
        static const std::string& idQState ();

        /**
         * @brief Identifier for specifying a State.
         * @return the identifier.
         */
        static const std::string& idTreeState ();

        /**
         * @brief Identifier for specifying a boolean.
         * @return the identifier.
         */
        static const std::string& idBoolean ();

        /**
         * @brief Identifier for specifying a double.
         * @return the identifier.
         */
        static const std::string& idDouble ();

        /**
         * @brief Identifier for specifying a float.
         * @return the identifier.
         */
        static const std::string& idFloat ();

        /**
         * @brief Identifier for specifying an integer.
         * @return the identifier.
         */
        static const std::string& idInteger ();

        /**
         * @brief Identifier for specifying a string.
         * @return the identifier.
         */
        static const std::string& idString ();

        /**
         * @brief Identifier for specifying a list of strings.
         * @return the identifier.
         */
        static const std::string& idStringList ();

        /**
         * @brief Identifier for specifying a list of integers.
         * @return the identifier.
         */
        static const std::string& idIntList ();

        /**
         * @brief Identifier for specifying a list of doubles.
         * @return the identifier.
         */
        static const std::string& idDoubleList ();

        /**
         * @brief Identifier for specifying a pair of strings.
         * @return the identifier.
         */
        static const std::string& idStringPair ();

        /**
         * @brief Identifier for the unit attribute.
         * @return the identifier.
         */
        static const std::string& idUnitAttribute ();

        /**
         * @brief Identifier for a Plane
         * @return the identifier.
         */
        static const std::string& idPlane ();

        /**
         * @brief Identifier for a Box
         * @return the identifier.
         */
        static const std::string& idBox ();

        /**
         * @brief Identifier for a Sphere
         * @return the identifier.
         */
        static const std::string& idSphere ();

        /**
         * @brief Identifier for a Cone
         * @return the identifier.
         */
        static const std::string& idCone ();

        /**
         * @brief Identifier for a Cylinder
         * @return the identifier.
         */
        static const std::string& idCylinder ();

        /**
         * @brief Identifier for a Tube
         * @return the identifier.
         * @return the identifier.
         */
        static const std::string& idTube ();

        /**
         * @brief Returns the conversion value for a given unit
         *
         * The value returned can be used to convert the values specified in the element to
         * the standard units used in RobWork.
         *
         * Throws an exception if \b key cannot be matched to a unit
         *
         * @param key [in] The key for which the get the unit
         * @return Conversion value
         */
        static double getUnit (const std::string key);

        /**
         * @brief Returns pair of strings from \b element
         *
         * Reads in \b element and returns a pair of strings.
         * If \b doCheckHeader = true it checks that the element tag name matches
         * XMLBasicTypes::StringPairId. Throws a rw::core::Exception if header does not match or if
         * failing to read two strings.
         *
         * @param element [in] Element to read
         * @param doCheckHeader [in] True if the element name should be checked
         * @return Pair of strings
         */
        static std::pair< std::string, std::string > readStringPair (rw::core::DOMElem::Ptr element,
                                                                     bool doCheckHeader = false);

        /**
         * @brief Reads in a list of string pairs that are childs of \b element
         *
         * Reads in all string pairs which are childs of the element \b element. This is
         * for example used when reading in DAF setups.
         *
         * Throws rw::core::Exception if failing to read and parse content.
         *
         * @param element [in] Element which string pairs as children
         * @return List of string pairs
         */
        static std::vector< std::pair< std::string, std::string > >
        readStringPairs (rw::core::DOMElem::Ptr element);

        /**
         * @brief Reads in a list of strings that are childs of \b element
         *
         * Reads in all strings which are childs of the element \b element.
         *
         * Throws rw::core::Exception if failing to read and parse content.
         *
         * @param element [in] Element which string pairs as children
         * @return List of strings
         */
        static std::vector< std::string > readStringList (rw::core::DOMElem::Ptr element);

        /**
         * @brief Reads in a string element
         *
         * Reads in \b element as a string element.
         * Throws a rw::core::Exception on error
         *
         * @param element [in] Element to parse
         * @param doCheckHeader [in] True if the element name should be checked
         * @return String giving the content of \b element
         */
        static std::string readString (rw::core::DOMElem::Ptr element, bool doCheckHeader = false);

        /**
         * @brief Reads in a double element
         *
         * Read in \b element and converts the content to a double
         * Throws a rw::core::Exception if failing to read or parse.
         *
         * @param element [in] Element to read in
         * @param doCheckHeader [in] True if the element name should be checked
         * @return double represented in \b element
         */
        static double readDouble (rw::core::DOMElem::Ptr element, bool doCheckHeader = false);

        /**
         * @brief Reads in a double element
         *
         * Read in \b element and converts the content to a double
         * Throws a rw::core::Exception if failing to read or parse.
         *
         * @param element [in] Element to read in
         * @param doCheckHeader [in] True if the element name should be checked
         * @return double represented in \b element
         */
        static float readFloat (rw::core::DOMElem::Ptr element, bool doCheckHeader = false);

        /**
         * @brief Reads in an integer element
         *
         * Read in \b element and converts the content to an integer
         * Throws a rw::core::Exception if failing to read or parse.
         *
         * @param element [in] Element to read in
         * @param doCheckHeader [in] True if the element name should be checked
         * @return int represented in \b element
         */
        static int readInt (rw::core::DOMElem::Ptr element, bool doCheckHeader = false);

        /**
         * @brief Reads in a list of integers from \b element
         *
         * Read in \b element and converts the content to a list of integers
         * Throws a rw::core::Exception if failing to read or parse.
         *
         * @param element [in] Element to read in
         * @param doCheckHeader [in] True if the element name should be checked
         * @return std::vector<int> represented in \b element
         */
        static std::vector< int > readIntList (rw::core::DOMElem::Ptr element,
                                               bool doCheckHeader = false);

        /**
         * @brief Reads in a list of doubles from \b element
         *
         * Read in \b element and converts the content to a list of doubles
         * Throws a rw::core::Exception if failing to read or parse.
         *
         * @param element [in] Element to read in
         * @param doCheckHeader [in] True if the element name should be checked
         * @return std::vector<double> represented in \b element
         */
        static std::vector< double > readDoubleList (rw::core::DOMElem::Ptr element,
                                                     bool doCheckHeader = false);

        /**
         * @brief Reads in a boolean element
         *
         * Read in \b element and converts the content to a bool. Only if the content of the
         * node equals "true" is true returned. Otherwise the method returns false.
         *
         * Throws a rw::core::Exception if failing to read or parse.
         *
         * @param element [in] Element to read in
         * @param doCheckHeader [in] True if the element name should be checked
         * @return bool represented in \b element
         */
        static bool readBool (rw::core::DOMElem::Ptr element, bool doCheckHeader = false);

        //------------------------ writing value of DOMElem

        /**
         * @brief Writes \b val to \b elem.
         * @param val [in] Value to write
         * @param elem [in] Element to which to write
         * @param addHeader [in] Whether or not to set the header of \b elem
         * @return newly created DOMElem
         */
        static rw::core::DOMElem::Ptr write (int val, rw::core::DOMElem::Ptr elem,
                                             bool addHeader = true);

        /**
         * @brief Writes \b val to \b elem.
         * @param val [in] Value to write
         * @param elem [in] Element to which to write
         * @param addHeader [in] Whether or not to set the header of \b elem
         * @return newly created DOMElem
         */
        static rw::core::DOMElem::Ptr write (double val, rw::core::DOMElem::Ptr elem,
                                             bool addHeader = true);

        /**
         * @brief Writes \b str to \b elem.
         * @param str [in] Value to write
         * @param elem [in] Element to which to write
         * @param addHeader [in] Whether or not to set the header of \b elem
         * @return newly created DOMElem
         */
        static rw::core::DOMElem::Ptr write (const std::string& str, rw::core::DOMElem::Ptr elem,
                                             bool addHeader = true);

        //------------------------ creating and writing DOMElem
        /**
         * @brief Create an element with name \b id and content \b value in the DOMDocument \b doc
         *
         * Create an element named \b id and adds a DOMText-node containing \b value as a child to
         * it.
         *
         * @param id [in] Name of the new element
         * @param value [in] Text value of the new element
         * @param doc [in] Document for which the new element shall be created
         *
         * @return The new element.
         */
        static rw::core::DOMElem::Ptr
        createElement (const std::string& id, const std::string& value, rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b value.
         *
         * Creates a DOMElement owned by \b doc and representing \b value
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param value [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createBoolean (bool value, rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b value.
         *
         * Creates a DOMElement owned by \b doc and representing \b value
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param value [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createDouble (double value, rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b value.
         *
         * Creates a DOMElement owned by \b doc and representing \b value
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param value [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createFloat (float value, rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b value.
         *
         * Creates a DOMElement owned by \b doc and representing \b value
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param value [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createInteger (int value, rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b string.
         *
         * Creates a DOMElement owned by \b doc and representing \b string
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param string [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createString (const std::string& string,
                                                    rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b strings.
         *
         * Creates a DOMElement owned by \b doc and representing \b strings
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param strings [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createStringList (const std::vector< std::string >& strings,
                                                        rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b ints.
         *
         * Creates a DOMElement owned by \b doc and representing \b ints
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param ints [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createIntList (const std::vector< int >& ints,
                                                     rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent \b doubles.
         *
         * Creates a DOMElement owned by \b doc and representing \b doubles
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param doubles [in] Value to represent
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createDoubleList (const std::vector< double >& doubles,
                                                        rw::core::DOMElem::Ptr doc);

        /**
         * @brief Creates an element to represent strings \b first and \b second.
         *
         * Creates a DOMElement owned by \b doc and representing strings \b first and \b second
         *
         * The method may throw a rw::core::Exception in case of errors
         *
         * @param first [in] First string in the pair
         * @param second [in] Second string in the pair
         * @param doc [in] Document which should contain the element
         * @return Pointer to the newly created DOMElement
         */
        static rw::core::DOMElem::Ptr createStringPair (const std::string& first,
                                                        const std::string& second,
                                                        rw::core::DOMElem::Ptr doc);

        /**
         * @brief Utility class which initializes local static variables.
         *
         * If the DOMBasisTypes is used outside main (as a part of global
         * initialization/destruction), the Initializer should be used explicitly to control the
         * static initialization/destruction order.
         *
         * Notice that the Initializer is automatically defined as a global variable, hence it
         * should not be necessary to specify the initializer explicitly if DOMBasisTypes is to be
         * used in local static initialization/destruction.
         */
        class Initializer
        {
          public:
            //! @brief Initializes when constructed.
            Initializer ();
        };

      private:
        static const Initializer initializer;

        DOMCoreBasisTypes (){};

        /*
         * Map used for mapping unit identifiers o their corresponding values
         */

        // static const UnitMap _Units;
    };

    /** @} */

}}        // namespace rw::core
#endif    // end include guard
