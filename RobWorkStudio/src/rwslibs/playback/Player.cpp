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

#include "Player.hpp"

#include "StateDraw.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/core/macros.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>
#include <rws/RobWorkStudio.hpp>

#include <boost/bind.hpp>
#include <chrono>
#include <thread>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::core;
using namespace rws;

Player::Player (TimedStatePath path, StateDrawPtr drawer, double tickInterval,
                RobWorkStudio* rwstudio) :
    _trajectory (TrajectoryFactory::makeLinearTrajectory (path)),
    _path (ownedPtr (new rw::trajectory::TimedStatePath (path))), _drawer (drawer),
    _tickInterval (tickInterval), _rwstudio (rwstudio), _record (false), _recNo (0),
    _rec_number_of_digits (2), _now (0), _direction (1), _velocityScale (1), _timer (this),
    _loop (false), _interpolate (true), _recordingOnly (false)
{
    RW_ASSERT (drawer);
    initialize ();
}

Player::Player (TimedStatePath::Ptr path, StateDrawPtr drawer, double tickInterval,
                RobWorkStudio* rwstudio) :
    _trajectory (TrajectoryFactory::makeLinearTrajectory (*path)),
    _path (path), _drawer (drawer), _tickInterval (tickInterval), _rwstudio (rwstudio),
    _record (false), _recNo (0), _rec_number_of_digits (2), _now (0), _direction (1),
    _velocityScale (1), _timer (this), _loop (false), _interpolate (true), _recordingOnly (false)
{
    RW_ASSERT (drawer);
    RW_ASSERT (path.isShared ());
    RW_ASSERT (!path.isNull ());
    initialize ();
}

Player::Player (double tickInterval, RobWorkStudio* rwstudio) :
    _trajectory (TrajectoryFactory::makeLinearTrajectory (TimedStatePath ())), _path (NULL),
    _tickInterval (tickInterval), _rwstudio (rwstudio), _record (false), _recNo (0),
    _rec_number_of_digits (2), _now (0), _direction (1), _velocityScale (1), _timer (this),
    _loop (false), _interpolate (true), _recordingOnly (true)
{
    initialize ();
}

void Player::initialize ()
{
    RW_ASSERT (_tickInterval > 0);
    connect (&_timer, SIGNAL (timeout ()), this, SLOT (tick ()));
    connect (&_recTimer, SIGNAL (timeout ()), this, SLOT (recordImage ()));
}

void Player::setTickInterval (double interval)
{
    _tickInterval = interval;
    if (_timer.isActive ()) {
        stopTimer ();
        startTimer ();
    }
}

unsigned int Player::calcLeadingZeros ()
{
    double number_of_frames = getEndTime () / _tickInterval;
    number_of_frames /= _velocityScale;
    return static_cast< unsigned int > (std::ceil (std::log10 (number_of_frames)));
}

void Player::setupRecording (const QString filename, const QString& type)
{
    _recordFilename = filename;
    _recordType     = type;
}

void Player::startRecording ()
{
    if (!_recordingOnly) {
        _rec_number_of_digits = calcLeadingZeros ();
    }
    _record = true;
    _recNo  = 0;
    startTimer ();
}

void Player::stopRecording ()
{
    stopTimer ();
    _record = false;
}

void Player::takeImage ()
{
    const int ms = (int) (_tickInterval * 1000 / 2);
    _recTimer.start (ms);
}

void Player::recordImage ()
{
    if (_record && _rwstudio != NULL) {
        // Create Filename
        QString number = QString::number (_recNo++);
        while ((unsigned int) number.length () < _rec_number_of_digits)
            number.prepend ("0");
        QString filename = _recordFilename + number + "." + _recordType;
        _rwstudio->saveViewGL (filename);
    }
    _recTimer.stop ();
}

void Player::tick ()
{
    const double end = getEndTime ();

    if (!_recordingOnly) {
        // Make sure that we do show the robot at the position at the start or end
        // of the path.
        bool outside = false;
        if (_now < 0) {
            _now    = 0;
            outside = true;
        }
        else if (end < _now) {
            _now    = end;
            outside = true;
        }

        // Draw the work cell.
        draw ();
        takeImage ();

        // If we reached the end and we are looping, then move the cursor to the
        // start or end.
        if (outside && _loop) {
            if (_direction < 0)
                _now = end;
            else
                _now = 0;
        }

        // If the range is empty or if we outside of the range and not looping:
        if (end <= 0 || (outside && !_loop)) {
            // then stop the player.
            stopTimer ();
        }
        else {    // Otherwise,
            // increment the time.
            _now += _direction * _velocityScale * _tickInterval;
        }
    }
    else {
        recordImage ();
    }
}

// Forward, backward and pause.

void Player::forward ()
{
    _direction = 1;
    runTimer ();
}

void Player::backward ()
{
    _direction = -1;
    runTimer ();
}

void Player::pauseOrResume ()
{
    if (timerIsRunning ())
        stopTimer ();
    else
        startTimer ();
}

// Move to start and end.

void Player::toStart ()
{
    _now = 0;
    stopTimer ();
    draw ();
}

void Player::toEnd ()
{
    _now = getEndTime ();
    stopTimer ();
    draw ();
}

void Player::step (bool forward)
{
    stopTimer ();

    if (_path->size () == 0)
        return;

    // Find the first index of the current segment
    const std::size_t N = _path->size () - 1;
    std::size_t curId   = 0;
    bool interpolated   = false;
    for (std::size_t i = 0; i < N; i++) {
        if ((*_path)[i].getTime () <= _now && _now < (*_path)[i + 1].getTime ()) {
            curId = i;
            if ((*_path)[i].getTime () != _now)
                interpolated = true;
            break;
        }
    }
    if (_now >= _path->back ().getTime ()) {
        curId = N;
    }

    // Determine next id
    std::size_t nextId = curId;
    if (forward)
        nextId++;
    else if (!interpolated && nextId > 0)
        nextId--;

    // Respect bounds
    if (nextId > N)
        nextId = N;

    // Move to new id
    _now = (*_path)[nextId].getTime ();
    _drawer->draw ((*_path)[nextId].getValue ());
    relativePositionChanged (_now / getEndTime ());
}

// Move to a specific time.

void Player::setRelativePosition (double relative)
{
    const double time = relative * getEndTime ();

    if (time < 0)
        toStart ();
    else if (getEndTime () < time)
        toEnd ();
    else {
        _now = time;
        // Should we stop the timer also?
        stopTimer ();
        draw ();
    }
}

// Change the speed.

void Player::setRelativeSpeed (double speed)
{
    _velocityScale = speed;
}

// Change the loop flag.
void Player::setLoopPlayback (bool loop)
{
    _loop = loop;
}

// Timer accessors.
void Player::runTimer ()
{
    if (!timerIsRunning ())
        startTimer ();
}

bool Player::timerIsRunning ()
{
    return _timer.isActive ();
}

void Player::startTimer ()
{
    const int ms = (int) (_tickInterval * 1000);
    _timer.start (ms);
}

void Player::stopTimer ()
{
    _timer.stop ();
}

void Player::draw ()
{
    // This takes care of empty trajectories.
    if (_interpolate) {
        if (0 <= _now && _now <= getEndTime ()) {
            _drawer->draw (_trajectory->x (_now));
        }
    }
    else {
        if (0 <= _now && _now <= getEndTime () && _path->size () > 0) {
            rw::trajectory::TimedStatePath& path = *_path;

            // Find State Closest to now
            double time = path[0].getTime ();
            double dt   = 0;

            for (unsigned int i = 0; i < path.size () - 1; i++) {
                if (i > 0) {
                    dt = path[i].getTime () - path[i - 1].getTime ();
                    if (dt < 0) {
                        dt = 0;
                    }
                }
                time += dt;
                double dtNext = path[i + 1].getTime () - path[i].getTime ();
                if (dtNext < 0) {
                    dtNext = 0;
                }

                if (time <= _now && _now <= time + dtNext) {
                    _drawer->draw (path[i].getValue ());
                    break;
                }
            }
        }
    }
    relativePositionChanged (_now / getEndTime ());
}

std::string Player::getInfoLabel () const
{
    if (getEndTime () < 0)
        return "No trajectory loaded";
    else {
        char buf[40];
        sprintf (buf, "%.2f", getEndTime ());

        return "Trajectory of length " + std::string (buf) + " s loaded";
    }
}

int Player::getPlayDirection ()
{
    return _direction;
}

// Constructors.
Player::Ptr Player::makeEmptyPlayer ()
{
    return ownedPtr (
        new Player (TimedStatePath (), makeEmptyStateDraw (), 1, (RobWorkStudio*) NULL));
}

Player::Ptr Player::makePlayer (const TimedStatePath& path, StateDrawPtr drawer,
                                double tickInterval, RobWorkStudio* rwstudio)
{
    return ownedPtr (new Player (path, drawer, tickInterval, rwstudio));
}
Player::Ptr Player::makePlayer (const TimedStatePath::Ptr path, StateDrawPtr drawer,
                                double tickInterval, RobWorkStudio* rwstudio)
{
    return ownedPtr (new Player (path, drawer, tickInterval, rwstudio));
}
