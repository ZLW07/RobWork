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

#ifndef RW_COLLISION_PROXIMITYSETUP_HPP
#define RW_COLLISION_PROXIMITYSETUP_HPP
#if !defined(SWIG)
#include "ProximitySetupRule.hpp"

#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>

#include <string>
#include <vector>
#endif
namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rw { namespace proximity {

    class CollisionSetup;

    /** @addtogroup proximity */
    /*@{*/
    //! @file rw/proximity/ProximitySetup.hpp
    /**
     * @brief Setup for the collision checker
     *
     * The ProximitySetup contains the rules about which frames should be
     * checked against each other
     */
    class ProximitySetup
    {
      public:
        /**
         * @brief Default constructor for when no excludes are described
         */
        ProximitySetup ();

        ProximitySetup (const CollisionSetup& csetup);

        /**
         @brief Constructs ProximitySetup with list of exclusions
             @param rules documentation missing !
             @cond
         @param exclude [in] pairs to be excluded
             @endcond
         */
        explicit ProximitySetup (const std::vector< rw::proximity::ProximitySetupRule >& rules);

        void addProximitySetupRule (const rw::proximity::ProximitySetupRule& rule);

        void removeProximitySetupRule (const rw::proximity::ProximitySetupRule& rule);

        /**
         * @brief Returns the exclude list
         * @return the exclude list
         */
        const std::vector< rw::proximity::ProximitySetupRule >& getProximitySetupRules () const { return _rules; }

        /**
         * @brief Combine setup of this and setup of \b b into this collision setup.
         */
        void merge (const ProximitySetup& setup, const std::string& prefix)
        {
            for (const ProximitySetupRule& rule : setup.getProximitySetupRules ()) {
                std::pair< std::string, std::string > patterns = rule.getPatterns ();
                if (prefix != "") {
                    patterns.first  = prefix + patterns.first;
                    patterns.second = prefix + patterns.second;
                }
                addProximitySetupRule (
                    ProximitySetupRule (patterns.first, patterns.second, rule.type ()));
            }
        }

        bool useExcludeStaticPairs () const { return _useExcludeStaticPairs; }

        void setUseExcludeStaticPairs (bool exclude) { _useExcludeStaticPairs = exclude; }

        bool useIncludeAll () const { return _useIncludeAll; }

        void setUseIncludeAll (bool includeAll) { _useIncludeAll = includeAll; }

        void setLoadedFromFile (bool loaded_from_file) { _loadedFromFile = loaded_from_file; }

        bool getLoadedFromFile () const { return _loadedFromFile; }

        void setFileName (const std::string& file_name) { _fileName = file_name; }

        std::string getFileName () const { return _fileName; }

        static ProximitySetup get (const rw::models::WorkCell& wc);
        static ProximitySetup get (rw::core::Ptr< rw::models::WorkCell > wc);
        static ProximitySetup get (const rw::core::PropertyMap& map);

        static void set (const ProximitySetup& setup, rw::core::Ptr< rw::models::WorkCell > wc);
        static void set (const ProximitySetup& setup, rw::core::PropertyMap& map);

      private:
        std::vector< rw::proximity::ProximitySetupRule > _rules;

        bool _useIncludeAll;
        bool _useExcludeStaticPairs;
        bool _loadedFromFile;
        std::string _fileName;
    };

    /*@}*/
}}    // namespace rw::proximity

#endif    // end include guard
