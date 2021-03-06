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

#include "StopCriteria.hpp"

#include <rw/common/Timer.hpp>
#include <rw/core/macros.hpp>

using namespace rw::pathplanning;
using namespace rw::common;
using namespace rw::core;

namespace {
class StopTime : public StopCriteria
{
  public:
    StopTime (double time) : _end (time) {}

  private:
    bool doStop () const
    {
        const double time = _timer.getTime ();
        if (time > _end)
            return true;
        else
            return false;
    }

    StopCriteria::Ptr doInstance () const { return ownedPtr (new StopTime (_end)); }

  private:
    double _end;
    Timer _timer;
};

class StopFixed : public StopCriteria
{
  public:
    StopFixed (bool value) : _value (value) {}

  private:
    bool doStop () const { return _value; }

    StopCriteria::Ptr doInstance () const { return ownedPtr (new StopFixed (_value)); }

    bool _value;
};

typedef boost::function< bool () > BoostFunction;

class StopByFun : public StopCriteria
{
  public:
    StopByFun (BoostFunction fun) : _fun (fun) {}

  private:
    bool doStop () const { return _fun (); }

    StopCriteria::Ptr doInstance () const { return ownedPtr (new StopByFun (_fun)); }

  private:
    BoostFunction _fun;
};

class StopByFlag : public StopCriteria
{
  public:
    StopByFlag (bool* flag) : _flag (flag) { RW_ASSERT (flag); }

  private:
    bool doStop () const { return *_flag; }

    StopCriteria::Ptr doInstance () const { return ownedPtr (new StopByFlag (_flag)); }

  private:
    bool* _flag;
};

class StopCnt : public StopCriteria
{
  public:
    StopCnt (int cnt) : _maxCnt (cnt), _cnt (0) {}

  private:
    bool doStop () const { return ++_cnt > _maxCnt; }

    StopCriteria::Ptr doInstance () const { return ownedPtr (new StopCnt (_maxCnt)); }

  private:
    int _maxCnt;
    mutable int _cnt;
};

class StopEither : public StopCriteria
{
  public:
    StopEither (const std::vector< StopCriteria::Ptr >& criteria) : _criteria (criteria) {}

  private:
    bool doStop () const
    {
        for (const StopCriteria::Ptr& stop : _criteria) {
            if (stop->stop ())
                return true;
        }
        return false;
    }

    StopCriteria::Ptr doInstance () const
    {
        std::vector< StopCriteria::Ptr > criteria;
        for (const StopCriteria::Ptr& stop : _criteria) {
            criteria.push_back (stop->instance ());
        }
        return ownedPtr (new StopEither (criteria));
    }

  private:
    std::vector< StopCriteria::Ptr > _criteria;
};
}    // namespace

//----------------------------------------------------------------------
// StopCriteria

bool StopCriteria::stop () const
{
    return doStop ();
}

StopCriteria::Ptr StopCriteria::instance () const
{
    return doInstance ();
}

//----------------------------------------------------------------------
// Constructors

StopCriteria::Ptr StopCriteria::stopAfter (double time)
{
    return ownedPtr (new StopTime (time));
}

StopCriteria::Ptr StopCriteria::stopNever ()
{
    return ownedPtr (new StopFixed (false));
}

StopCriteria::Ptr StopCriteria::stopNow ()
{
    return ownedPtr (new StopFixed (true));
}

StopCriteria::Ptr StopCriteria::stopByFlag (bool* stop)
{
    return ownedPtr (new StopByFlag (stop));
}

StopCriteria::Ptr StopCriteria::stopByFun (BoostFunction fun)
{
    return ownedPtr (new StopByFun (fun));
}

StopCriteria::Ptr StopCriteria::stopCnt (int cnt)
{
    return ownedPtr (new StopCnt (cnt));
}

StopCriteria::Ptr StopCriteria::stopEither (const std::vector< StopCriteria::Ptr >& criteria)
{
    return ownedPtr (new StopEither (criteria));
}

StopCriteria::Ptr StopCriteria::stopEither (const StopCriteria::Ptr& a, const StopCriteria::Ptr& b)
{
    std::vector< StopCriteria::Ptr > criteria;
    criteria.push_back (a);
    criteria.push_back (b);
    return stopEither (criteria);
}
