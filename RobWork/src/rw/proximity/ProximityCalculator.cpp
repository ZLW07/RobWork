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

#include "ProximityCalculator.hpp"

#include <rw/common/ScopedTimer.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/Object.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

#include <float.h>
#include <regex>
#include <vector>

using namespace rw::common;
using namespace rw::core;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::geometry;

namespace {
// Find the frame or crash.
std::vector< std::string > searchName (const std::map< std::string, Frame* >& frameMap,
                                       const std::string& frameName)
{
    std::string regex_pattern = frameName;
    std::regex reg (regex_pattern);
    std::vector< std::string > ret;
    for (const std::pair< std::string, Frame* >& x : frameMap) {
        if (std::regex_match (x.first, reg)) {
            ret.push_back (x.first);
        }
    }
    return ret;
}

Frame* lookupFrame (const std::map< std::string, Frame* >& frameMap, const std::string& frameName)
{
    const std::map< std::string, Frame* >::const_iterator pos = frameMap.find (frameName);
    if (pos == frameMap.end ())
        RW_THROW ("Frame " << StringUtil::quote (frameName) << " is not present in frame map.");

    return pos->second;
}

bool isInList (const FramePair& pair, const FramePairList& pairs)
{
    return std::find (pairs.cbegin (), pairs.cend (), pair) != pairs.cend ();
}
}    // namespace

namespace rw { namespace proximity {

    template< class T >
    ProximityCalculator< T >::ProximityCalculator (rw::core::Ptr< Frame > root,
                                                   rw::core::Ptr< WorkCell > workcell,
                                                   rw::core::Ptr< Strategy > strategy,
                                                   const State& initial_state) :
        _strategy (strategy),
        _state (initial_state), _root (root)
    {
        RW_ASSERT (strategy != NULL);
        RW_ASSERT (workcell != NULL);
        _proxFilterStrat = ownedPtr (new BasicFilterStrategy (workcell));
        try {
            _setup = ownedPtr (new CollisionSetup ());
            _setup->merge (CollisionSetup::get (workcell));
        }
        catch (const Exception& exp) {
            RW_WARN (exp.what ());
        }
        initGeom (workcell);
        initDistPairs (initial_state);

        this->resetComputationTimeAndCount ();
    }

    template< class T >
    ProximityCalculator< T >::ProximityCalculator (rw::core::Ptr< WorkCell > workcell,
                                                   rw::core::Ptr< Strategy > strategy) :
        _strategy (strategy),
        _state (workcell->getDefaultState ()), _root (workcell->getWorldFrame ())
    {
        RW_ASSERT (workcell != NULL);

        _proxFilterStrat = ownedPtr (new BasicFilterStrategy (workcell));
        try {
            _setup = ownedPtr (new CollisionSetup ());
            _setup->merge (CollisionSetup::get (workcell));
        }
        catch (const Exception& exp) {
            RW_WARN (exp.what ());
        }
        if (!strategy.isNull ()) {
            initGeom (workcell);
            initDistPairs (workcell->getDefaultState ());
        }
        this->resetComputationTimeAndCount ();
    }

    template< class T >
    void ProximityCalculator< T >::setStrategy (rw::core::Ptr< Strategy > strategy)
    {
        RW_ASSERT (strategy);
        _strategy = strategy;
    }

    template< class T >
    ProximityStrategyData ProximityCalculator< T >::calculate (
        const rw::kinematics::State& state, rw::core::Ptr< ProximityStrategyData > settings,
        rw::core::Ptr< std::vector< ProximityStrategyData > > result)
    {
        RW_WARN ("calculate, is not implemented for the chosen strategy Type");
        return ProximityStrategyData ();
    }

    template<>
    ProximityStrategyData ProximityCalculator< CollisionStrategy >::calculate (
        const rw::kinematics::State& state, rw::core::Ptr< ProximityStrategyData > settings,
        rw::core::Ptr< std::vector< ProximityStrategyData > > result)
    {
        ScopedTimer stimer (_timer);
        _numberOfCalls++;

        if (result != NULL) {
            result->clear ();
        }
        // first we update the broadphase filter with the current state
        ProximityFilter::Ptr filter = _proxFilterStrat->update (state);
        FKTable fk (state);
        ProximityStrategyData data;
        if (!settings.isNull ()) {
            data = *settings;
        }
        bool gotFirstContact = false;
        ProximityStrategyData firstContact;
        bool stopAtFirstContact = result.isNull ();

        // next we query the BP filter for framepairs that are possibly in collision
        while (!filter->isEmpty ()) {
            const FramePair& pair = filter->frontAndPop ();
            // and lastly we use the dispatcher to find the strategy the
            // is required to compute the narrowphase collision
            const ProximityModel::Ptr& a = _frameToModels[*pair.first];
            const ProximityModel::Ptr& b = _frameToModels[*pair.second];

            if (a == NULL || b == NULL)
                continue;

            const Transform3D<> aT = fk.get (*pair.first);
            const Transform3D<> bT = fk.get (*pair.second);
            bool res               = _strategy.isNull ();
            if (!res) {
                res = _strategy->inCollision (a, aT, b, bT, data);
                if (res && !result.isNull ()) {
                    result->push_back (data);
                }
            }
            if (res) {
                if (stopAtFirstContact) {
                    return data;
                }
                else if (!gotFirstContact) {
                    gotFirstContact = true;
                    firstContact    = data;
                }
            }
        }
        return firstContact;
    }

    template<>
    ProximityStrategyData ProximityCalculator< DistanceStrategy >::calculate (
        const rw::kinematics::State& state, rw::core::Ptr< ProximityStrategyData > settings,
        rw::core::Ptr< std::vector< ProximityStrategyData > > result)
    {
        ScopedTimer stimer (_timer);
        _numberOfCalls++;
        RW_ASSERT (_strategy != NULL);
        FKTable fk (state);

        if (settings != NULL) {
            RW_THROW ("Proximity Calculator<DistanceStrategy>::calculate settings are ignored");
        }

        if (result != NULL) {
            result->clear ();
        }

        ProximityStrategyData data;
        DistanceResult distance;
        distance.distance = DBL_MAX;
        // std::cout<<"Distance Pairs = "<<_distancePairs.size()<<std::endl;
        const int N = (int) _distancePairs.size ();
        int i;
        Transform3D<> ta, tb;

#ifdef RW_HAVE_OMP
#pragma omp parallel for shared(distance, result, fk) private(i, data, ta, tb) schedule(static, 1)
#endif
        for (i = 0; i < N; i++) {
            Frame::Ptr a = _distancePairs[i].first;
            Frame::Ptr b = _distancePairs[i].second;

            DistanceResult dist;

#ifdef RW_HAVE_OMP
#pragma omp critical
#endif
            {
                ta = fk.get (*a);
                tb = fk.get (*b);
            }    //#End #pragma omp critical

            if (distance.distance == DBL_MAX || _thresholdStrategy == NULL || result != NULL) {
                dist = _strategy->distance (a.get (), ta, b.get (), tb, data);
            }
            else {
                dist = _thresholdStrategy->distance (
                    a.get (), ta, b.get (), tb, distance.distance, data);
            }

            dist.f1 = a.get ();
            dist.f2 = b.get ();

#ifdef RW_HAVE_OMP
#pragma omp critical
#endif
            {
                if (dist.distance < distance.distance) {
                    distance = dist;
                }

                if (result != NULL) {
                    data.getDistanceData () = dist;
                    result->push_back (data);
                }

            }    //#End #pragma omp critical
        }
        //	} //End #pragma omp parallel for schedule
        data.getDistanceData () = distance;
        return data;
    }

    template<>
    ProximityStrategyData ProximityCalculator< DistanceMultiStrategy >::calculate (
        const rw::kinematics::State& state, rw::core::Ptr< ProximityStrategyData > settings,
        rw::core::Ptr< std::vector< ProximityStrategyData > > result)
    {
        ScopedTimer stimer (_timer);
        _numberOfCalls++;
        RW_ASSERT (_strategy != NULL);

        FKTable fk (state);
        double tolerance = DBL_MAX;
        if (settings != NULL) {
            tolerance = settings->getMultiDistanceTolerance ();
        }

        if (result != NULL) {
            result->clear ();
        }

        ProximityStrategyData data;
        MultiDistanceResult distance;
        distance.distance = DBL_MAX;
        const int N       = (int) _distancePairs.size ();
        int i;
        Transform3D<> ta, tb;

#ifdef RW_HAVE_OMP
#pragma omp parallel for shared(distance, result, fk) private(i, data, ta, tb) schedule(static, 1)
#endif
        for (i = 0; i < N; i++) {
            Frame::Ptr a = _distancePairs[i].first;
            Frame::Ptr b = _distancePairs[i].second;

            MultiDistanceResult dist;

#ifdef RW_HAVE_OMP
#pragma omp critical
#endif
            {
                ta = fk.get (*a);
                tb = fk.get (*b);
            }    //#End #pragma omp critical

            dist = _strategy->distances (a.get (), ta, b.get (), tb, tolerance, data);

#ifdef RW_HAVE_OMP
#pragma omp critical
#endif
            {
                if (dist.distance < distance.distance) {
                    distance = dist;
                }

                if (result != NULL) {
                    data.getMultiDistanceData () = dist;
                    result->push_back (data);
                }

            }    //#End #pragma omp critical
        }
        //	} //End #pragma omp parallel for schedule
        data.getMultiDistanceData () = distance;
        return data;
    }

    template< class T >
    bool ProximityCalculator< T >::addGeometry (rw::core::Ptr< Frame > frame,
                                                const rw::core::Ptr< Geometry >& geometry)
    {
        if (geometry.isNull ()) {
            RW_THROW ("Unable to add NULL as geometry");
        }
        bool res = true;
        if (!_strategy.isNull ()) {
            res = _strategy->addModel (frame.get (), geometry);
            if (res) {
                _frameToModels[*frame] = _strategy->getModel (frame.get ());
                initDistPairs (_state);
                _proxFilterStrat->addGeometry (frame.get (), geometry);
            }
        }
        else {
            _proxFilterStrat->addGeometry (frame.get (), geometry);
        }

        return res;
    }

    template< class T >
    void ProximityCalculator< T >::removeGeometry (rw::core::Ptr< Frame > frame,
                                                   const rw::core::Ptr< Geometry >& geometry)
    {
        removeGeometry (frame, geometry->getId ());
    }

    template< class T >
    void ProximityCalculator< T >::removeGeometry (rw::core::Ptr< Frame > frame,
                                                   const std::string geometryId)
    {
        _proxFilterStrat->removeGeometry (frame.get (), geometryId);
        if (!_strategy.isNull ()) {
            if (!_strategy->hasModel (frame.get ())) {
                RW_THROW ("Frame does not have any proximity models attached!");
            }

            ProximityModel::Ptr model = _strategy->getModel (frame.get ());
            if (!model.isNull ()) {
                model->removeGeometry (geometryId);
                _frameToModels[*frame] = _strategy->getModel (frame.get ());
            }
        }
    }

    template< class T > void ProximityCalculator< T >::addRule (const ProximitySetupRule& rule)
    {
        _proxFilterStrat->addRule (rule);
        // TODO(kalor) should this be applied to treshold strategy aswell
    }

    template< class T > void ProximityCalculator< T >::removeRule (const ProximitySetupRule& rule)
    {
        _proxFilterStrat->removeRule (rule);
        // TODO(kalor) should this be applied to treshold strategy aswell
    }

    template< class T >
    std::vector< std::string >
    ProximityCalculator< T >::getGeometryIDs (rw::core::Ptr< Frame > frame)
    {
        if (!_frameToModels.has (*frame)) {
            return std::vector< std::string > ();
        }
        ProximityModel::Ptr model = _frameToModels[*frame];
        if (model == NULL) {
            return std::vector< std::string > ();
        }
        return model->getGeometryIDs ();
    }

    template< class T >
    bool ProximityCalculator< T >::hasGeometry (rw::core::Ptr< Frame > frame,
                                                const std::string& geometryId)
    {
        std::vector< std::string > names = getGeometryIDs (frame);
        for (const std::string& name : names) {
            if (name == geometryId)
                return true;
        }
        return false;
    }

    template< class T >
    rw::core::Ptr< Geometry > ProximityCalculator< T >::getGeometry (rw::core::Ptr< Frame > frame,
                                                                     const std::string& geometryId)
    {
        if (!_frameToModels.has (*frame)) {
            return NULL;
        }

        ProximityModel::Ptr model = _frameToModels[*frame];
        if (model == NULL) {
            return NULL;
        }

        for (rw::core::Ptr< Geometry >& geom : model->getGeometries ()) {
            if (geom->getId () == geometryId)
                return geom;
        }
        return NULL;
    }

    template< class T >
    void ProximityCalculator< T >::initGeom (rw::core::Ptr< rw::models::WorkCell > wc)
    {
        std::vector< Object::Ptr > objects = wc->getObjects ();
        State state                        = wc->getDefaultState ();
        for (Object::Ptr object : objects) {
            for (Geometry::Ptr geom : object->getGeometry (state)) {
                Frame* frame = geom->getFrame ();
                RW_ASSERT (frame);
                _strategy->addModel (frame, geom);
                _frameToModels[*frame] = _strategy->getModel (frame);
            }
        }
    }

    template<> void ProximityCalculator< CollisionStrategy >::initDistPairs (const State& state) {
        //Not nessesary for collision strategy
    }
    template< class T > void ProximityCalculator< T >::initDistPairs (const State& state)
    {
        _thresholdStrategy = _strategy;

        _distancePairs.clear ();

        // All frames reachable from the root.
        const std::vector< Frame* >& frames = Kinematics::findAllFrames (_root.get (), state);

        // All pairs of frames.
        FramePairList pairs;
        typedef std::vector< Frame* >::const_iterator I;
        for (I from = frames.cbegin (); from != frames.cend (); ++from) {
            if (_strategy->hasModel (*from)) {
                I to = from;
                for (++to; to != frames.cend (); ++to) {
                    if (_strategy->hasModel (*to)) {
                        pairs.push_back (FramePair (*from, *to));
                    }
                }
            }
        }

        // All pairs of frames to exclude.
        FramePairList exclude_pairs;
        const std::map< std::string, Frame* >& frameMap =
            Kinematics::buildFrameMap (_root.get (), state);
        StringPairList exclude = _setup->getExcludeList ();

        typedef StringPairList::iterator EI;
        for (EI p = exclude.begin (); p != exclude.end (); ++p) {
            if (p->first.find_first_of ("*") != std::string::npos ||
                p->second.find_first_of ("*") != std::string::npos) {
                std::vector< std::string > matches_left  = searchName (frameMap, p->first);
                std::vector< std::string > matches_right = searchName (frameMap, p->second);

                for (std::string& ml : matches_left) {
                    for (std::string& mr : matches_right) {
                        Frame* first  = lookupFrame (frameMap, ml);
                        Frame* second = lookupFrame (frameMap, mr);
                        exclude_pairs.push_back (FramePair (first, second));
                        exclude_pairs.push_back (FramePair (second, first));
                    }
                }
            }
            else {
                Frame* first  = lookupFrame (frameMap, p->first);
                Frame* second = lookupFrame (frameMap, p->second);
                exclude_pairs.push_back (FramePair (first, second));
                exclude_pairs.push_back (FramePair (second, first));
            }
        }

        // Include in the final list only the pairs that are not present in the
        // exclude list.
        typedef FramePairList::const_iterator PLI;
        for (PLI p = pairs.cbegin (); p != pairs.cend (); ++p) {
            if (!isInList (*p, exclude_pairs))
                _distancePairs.push_back (*p);
        }
    }

    template class ProximityCalculator< DistanceMultiStrategy >;
    template class ProximityCalculator< DistanceStrategy >;
    template class ProximityCalculator< CollisionStrategy >;

}}    // namespace rw::proximity