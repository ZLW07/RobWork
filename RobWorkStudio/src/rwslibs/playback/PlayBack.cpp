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

#include "PlayBack.hpp"

#include "Player.hpp"
#include "StateDraw.hpp"

#include <rw/core/Exception.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/path/PathLoaderCSV.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QSlider>
#include <QToolBar>
#include <QVBoxLayout>
#include <boost/bind.hpp>

using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::core;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::models;

using namespace rws;
namespace {
const int sliderEnd = 4000;
}

//----------------------------------------------------------------------
// Standard plugin methods

// Other useful methods:
//   QToolBar::addSeparator();

PlayBack::PlayBack () :
    RobWorkStudioPlugin ("PlayBack", QIcon (":/playback.png")), _workcell (0),
    _player (Player::makeEmptyPlayer ()), _inSliderSet (false), _inRelativePositionChanged (false)
{
    // Construct widget and layout for QDockWidget
    QWidget* widg    = new QWidget (this);
    QVBoxLayout* lay = new QVBoxLayout (widg);
    widg->setLayout (lay);
    this->setWidget (widg);

    // The playback buttons.

    QToolBar* playToolbar = new QToolBar ();
    lay->addWidget (playToolbar);    // own playToolbar

    playToolbar->setSizePolicy (QSizePolicy (QSizePolicy::Preferred, QSizePolicy::Fixed));

    {
        QAction* act = playToolbar->addAction (QIcon (":/fileopen.png"), "Open playback file");
        connect (act, SIGNAL (triggered ()), this, SLOT (openPath ()));
    }

    {
        QAction* act = playToolbar->addAction (QIcon (":/filesave.png"), "Save Path");
        connect (act, SIGNAL (triggered ()), this, SLOT (savePath ()));
    }

    {
        QAction* act = playToolbar->addAction (QIcon (":/start.png"), "Start of trajectory");
        connect (act, SIGNAL (triggered ()), this, SLOT (toStartPlay ()));
    }

    _backward = playToolbar->addAction (QIcon (":/backward.png"), "Backward play");
    connect (_backward, SIGNAL (triggered ()), this, SLOT (backwardPlay ()));

    _pauseAndResume = playToolbar->addAction (QIcon (":/pause.png"), "Pause play");
    _pauseAndResume->setShortcut (Qt::Key_Space);
    connect (_pauseAndResume, SIGNAL (triggered ()), this, SLOT (pauseOrResumePlay ()));

    _forward = playToolbar->addAction (QIcon (":/forward.png"), "Forward play");
    connect (_forward, SIGNAL (triggered ()), this, SLOT (forwardPlay ()));

    if (_player->getPlayDirection () < 0) {
        _backward->setVisible (false);
        _pauseAndResume->setIcon (QIcon (":/backward_pause.png"));
    }
    else if (_player->getPlayDirection () > 0) {
        _forward->setVisible (false);
        _pauseAndResume->setIcon (QIcon (":/forward_pause.png"));
    }

    {
        QAction* act = playToolbar->addAction (QIcon (":/end.png"), "End of trajectory");
        connect (act, SIGNAL (triggered ()), this, SLOT (toEndPlay ()));
    }

    {
        QAction* act = playToolbar->addAction (QIcon (":/reload.png"), "Reload playback file");
        connect (act, SIGNAL (triggered ()), this, SLOT (reloadPlay ()));
    }

    {
        QAction* act =
            playToolbar->addAction (QIcon (":/record_images.png"), "Record images to file");
        act->setCheckable (true);
        act->setChecked (false);
        connect (act, SIGNAL (triggered (bool)), this, SLOT (record (bool)));
    }

    {
        QIcon icon (":/settings.png");
        QAction* act = playToolbar->addAction (QIcon (":/settings.png"), "Playback Settings");
        connect (act, SIGNAL (triggered ()), this, SLOT (showSettings ()));
    }

    {
        _loop = new QCheckBox ("Loop playback");
        playToolbar->addWidget (_loop);    // own loop.
        connect (_loop, SIGNAL (stateChanged (int)), this, SLOT (loopPlaybackChanged (int)));
    }

    {
        // The velocity of playback.

        playToolbar->addWidget (new QLabel (" Speed: "));
        _speed = new QDoubleSpinBox ();
        playToolbar->addWidget (_speed);    // own speed.
        _speed->setRange (0, 999);
        _speed->setDecimals (0);
        _speed->setSuffix (" %");
        _speed->setValue (100);

        connect (_speed, SIGNAL (valueChanged (double)), this, SLOT (speedValueChanged (double)));
    }
    {
        _interpolate = new QCheckBox ("Interpolate");
        _interpolate->setCheckState (Qt::Checked);
        playToolbar->addWidget (_interpolate);    // own loop.
        connect (_interpolate, SIGNAL (stateChanged (int)), this, SLOT (interpolateChanged (int)));
    }

    // A slider for changing the position.

    {
        _slider = new QSlider (Qt::Horizontal);
        lay->addWidget (_slider);    // own _slider.

        _slider->setRange (0, sliderEnd);

        // The normalized step size for Page Up / Page Down keys.
        const double pageScale = 0.10;
        const int pageStep     = (int) (pageScale * (_slider->maximum () - _slider->minimum ()));
        _slider->setPageStep (pageStep);

        connect (_slider, SIGNAL (valueChanged (int)), this, SLOT (sliderSetPosition (int)));
    }

    // Info label.

    _info = new QLabel ("");
    lay->addWidget (_info);    // own _info.
    lay->setAlignment (_info, Qt::AlignTop);

    setInfoLabel ();
}

PlayBack::~PlayBack ()
{}

void PlayBack::initialize ()
{
    getRobWorkStudio ()->stateTrajectoryPtrChangedEvent ().add (
        boost::bind (&PlayBack::stateTrajectoryChangedListener, this, boost::arg< 1 > ()), this);
    connect(this,SIGNAL(stateTrajectoryChangedSignal()),this,SLOT(stateTrajectoryChanged()));
}

void PlayBack::open (WorkCell* workcell)
{
    close ();
    _workcell = workcell;
    //_workcellGLDrawer = getRobWorkStudio()->getWorkCellGLDrawer();
}

void PlayBack::close ()
{
    _workcell = 0;
    //_workcellGLDrawer = 0;

    _player = Player::makeEmptyPlayer ();
    _file   = "";

    setInfoLabel ();
}

void PlayBack::setInfoLabel ()
{
    _info->setText (_player->getInfoLabel ().c_str ());
}

//----------------------------------------------------------------------
// How the work cell is drawn.

class PlayBack::MyStateDraw : public StateDraw
{
  public:
    MyStateDraw (PlayBack* owner) : _owner (owner) {}

    void draw (const State& state) const { _owner->draw (state); }

  private:
    PlayBack* _owner;
};

void PlayBack::draw (const State& state)
{
    getRobWorkStudio ()->setState (state);
}

Ptr< StateDraw > PlayBack::makeMyStateDraw ()
{
    return ownedPtr (new MyStateDraw (this));
}

//----------------------------------------------------------------------
// Slots
namespace {
const double timerInterval = 1.0 / 50;
}
void PlayBack::record (bool record)
{
    if (_player.get () == NULL || _player->_path.get () == NULL || _player->_path->size () == 0) {
        // create a dummy player
        _player = ownedPtr (new Player (1.0 / 5.0, getRobWorkStudio ()));
    }

    if (record) {
        _player->setupRecording (_settings.getRecordFilename (), _settings.getRecordFileType ());
        _player->startRecording ();
    }
    else {
        _player->stopRecording ();
    }
}

void PlayBack::showSettings ()
{
    _settings.exec ();
    _player->setTickInterval (_settings.getUpdateRate ());
    _player->setupRecording (_settings.getRecordFilename (), _settings.getRecordFileType ());
    _player->setRelativeSpeed (_settings.getScale ());
}

void PlayBack::sliderSetPosition (int val)
{
    _inSliderSet = true;

    // Draw the work cell.
    if (!_inRelativePositionChanged) {
        _player->setRelativePosition ((double) val / sliderEnd);
        getRobWorkStudio ()->genericAnyEvent ().fire ("PlayBack::TimeRelative",
                                                      (double) val / sliderEnd);
    }

    _inRelativePositionChanged = false;
}

void PlayBack::relativePositionChanged (double relative)
{
    _inRelativePositionChanged = true;

    // Move the slider.
    if (!_inSliderSet) {
        _slider->setValue ((int) (relative * sliderEnd));
        getRobWorkStudio ()->genericAnyEvent ().fire ("PlayBack::TimeRelative", relative);
    }

    _inSliderSet = false;
}

void PlayBack::speedValueChanged (double percent)
{
    _player->setRelativeSpeed (percent / 100);
}

void PlayBack::loopPlaybackChanged (int state)
{
    _player->setLoopPlayback (state != 0);
}

void PlayBack::interpolateChanged (int state)
{
    _player->setInterpolate (state != 0);
}

void PlayBack::openPath ()
{
    // Load a .rwplay file.

    // Get a file name:
    const QString dir (_previousOpenSaveDirectory.c_str ());
    QString selectedFilter;
    QString filename = QFileDialog::getOpenFileName (this,
                                                     "Open playback file",    // Title
                                                     dir,                     // Directory
                                                     "Playback files ( *.rwplay )"
                                                     " \n Comma separated values ( *.csv )"
                                                     " \n All ( *.* )",
                                                     &selectedFilter);

    if (!filename.isEmpty ()) {
        _previousOpenSaveDirectory = StringUtil::getDirectoryName (filename.toStdString ());
        openPlayFile (filename.toStdString ());
    }
}

void PlayBack::savePath ()
{
    const QString dir (_previousOpenSaveDirectory.c_str ());
    QString filename = QFileDialog::getSaveFileName (
        this, "Save playback file", dir, "Playback files ( *.rwplay )");

    if (!filename.isEmpty ()) {
        _previousOpenSaveDirectory = StringUtil::getDirectoryName (filename.toStdString ());

        if (StringUtil::getFileExtension (filename.toStdString ()) != ".rwplay")
            filename += ".rwplay";

        PathLoader::storeTimedStatePath (
            *_workcell, getRobWorkStudio ()->getTimedStatePath (), filename.toStdString ());
    }
}

void PlayBack::openPlayFile (const std::string& file)
{
    // Check file extension:
    std::string filetype = file.substr (file.find_last_of ('.', file.length ()));

    if (!filetype.compare (".csv")) {
        try {
            csvOpenPlayFile (file);
        }
        catch (const Exception& exc) {
            std::stringstream buf;
            buf << "Can't open playback file in CSV format";

            QMessageBox::information (
                NULL, buf.str ().c_str (), exc.getMessage ().getText ().c_str (), QMessageBox::Ok);

            // We shouldn't need to have to close() anything. csvOpenPlayFile()
            // should behave sensibly with respect to exceptions.
        }
    }
    else if (!filetype.compare (".rwplay")) {
        try {
            rawOpenPlayFile (file);
        }
        catch (const Exception& exc) {
            std::stringstream buf;
            buf << "Can't open playback file in rwplay format";

            QMessageBox::information (
                NULL, buf.str ().c_str (), exc.getMessage ().getText ().c_str (), QMessageBox::Ok);

            // We shouldn't need to have to close() anything. rawOpenPlayFile()
            // should behave sensibly with respect to exceptions.
        }
    }
    else {
        RW_THROW ("Unknown file extension - expected either .csv or .rwplay!");
    }
}

void PlayBack::stateTrajectoryChangedListener (const TimedStatePath::Ptr path)
{
    stateTrajectoryChangedSignal();
}
void PlayBack::stateTrajectoryChanged()
{   
    TimedStatePath::Ptr path = getRobWorkStudio ()->getTimedStatePathPtr();
    if (!path->empty ()) {
        // Reset the player.
        _player = Player::makePlayer (path, makeMyStateDraw (), timerInterval, getRobWorkStudio ());

        setInfoLabel ();
        connect (_player.get (),
                 SIGNAL (relativePositionChanged (double)),
                 this,
                 SLOT (relativePositionChanged (double)));

        _slider->setValue (0);

        sliderSetPosition (_slider->value ());

        speedValueChanged (_speed->value ());

        loopPlaybackChanged (_loop->checkState ());
        interpolateChanged (_interpolate->checkState ());
    }
    else {
        _player = Player::makeEmptyPlayer ();
        setInfoLabel ();
    }
}

void PlayBack::rawOpenPlayFile (const std::string& file)
{
    if (!file.empty () && _workcell) {
        // Load the sequence of states.
        TimedStatePath path = PathLoader::loadTimedStatePath (*_workcell, file);

        getRobWorkStudio ()->setTimedStatePath (path);

        // Store the file name.
        _file = file;
    }
}
void PlayBack::csvOpenPlayFile (const std::string& file)
{
    if (!file.empty () && _workcell) {
        // Load the sequence of states.
        TimedStatePath path = PathLoaderCSV::loadTimedStatePath (*_workcell, file);

        getRobWorkStudio ()->setTimedStatePath (path);

        // Store the file name.
        _file = file;
    }
}

void PlayBack::forwardPlay ()
{
    if (_player->getPlayDirection () < 0) {
        _backward->setVisible (true);
        _forward->setVisible (false);
        _pauseAndResume->setIcon (QIcon (":/forward_pause.png"));
    }
    _player->forward ();
}

void PlayBack::backwardPlay ()
{
    if (_player->getPlayDirection () > 0) {
        _backward->setVisible (false);
        _forward->setVisible (true);
        _pauseAndResume->setIcon (QIcon (":/backward_pause.png"));
    }
    _player->backward ();
}

void PlayBack::pauseOrResumePlay ()
{
    _player->pauseOrResume ();
}

void PlayBack::toStartPlay ()
{
    _player->toStart ();
}

void PlayBack::toEndPlay ()
{
    _player->toEnd ();
}

void PlayBack::reloadPlay ()
{
    if (_file.empty ())
        openPath ();
    else
        openPlayFile (_file);
}

void PlayBack::keyPressEvent (QKeyEvent* e)
{
    if (e->key () == Qt::Key_F)
        _player->step (true);
    else if (e->key () == Qt::Key_B)
        _player->step (false);
}
