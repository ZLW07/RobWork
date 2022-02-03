/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "DistanceMultiResultWidget.hpp"

#include "ui_DistanceResultWidget.h"

#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderGeometry.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogDistanceMultiResult.hpp>
#include <rwsim/log/LogPositions.hpp>
using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::models;
using rwlibs::opengl::RenderGeometry;
using rwsim::dynamics::DynamicWorkCell;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

DistanceMultiResultWidget::DistanceMultiResultWidget (const LogDistanceMultiResult::CPtr entry,
                                                      QWidget* parent) :
    SimulatorLogEntryWidget (parent),
    _ui (new Ui::DistanceResultWidget ()), _positions (entry->getPositions ()), _result (entry)
{
    _ui->setupUi (this);
    connect (_ui->_framePairTable->selectionModel (),
             SIGNAL (selectionChanged (const QItemSelection&, const QItemSelection&)),
             this,
             SLOT (framePairsChanged (const QItemSelection&, const QItemSelection&)));

    QStringList headerLabels;
    headerLabels.push_back ("First");
    headerLabels.push_back ("Second");
    headerLabels.push_back ("Distance");
    _ui->_framePairTable->setHorizontalHeaderLabels (headerLabels);
}

DistanceMultiResultWidget::~DistanceMultiResultWidget ()
{
    if (_root != NULL) {
        _root->removeChild ("DistancesMulti");
    }
}

void DistanceMultiResultWidget::setDWC (const DynamicWorkCell::CPtr dwc)
{
    _dwc = dwc;
}

void DistanceMultiResultWidget::setEntry (const SimulatorLog::CPtr entry)
{
    const rw::common::Ptr< const LogDistanceMultiResult > set =
        entry.cast< const LogDistanceMultiResult > ();
    if (!(set == NULL))
        _result = set;
    else
        RW_THROW ("DistanceMultiResultWidget (setEntry): invalid entry!");
}

SimulatorLog::CPtr DistanceMultiResultWidget::getEntry () const
{
    return _result;
}

void DistanceMultiResultWidget::updateEntryWidget ()
{
    const std::vector< LogDistanceMultiResult::ResultInfo >& results = _result->getResults ();

    const int nrOfEntries = static_cast< int > (results.size ());
    if (results.size () > static_cast< std::size_t > (nrOfEntries))
        RW_THROW ("There are too many entries for the widget to handle!");

    _ui->_description->setText (QString::fromStdString (_result->getDescription ()));

    _ui->_framePairTable->setRowCount (nrOfEntries);
    _ui->_framePairTable->setSortingEnabled (false);

    int row = 0;
    for (const LogDistanceMultiResult::ResultInfo& info : results) {
        const std::string nameA = (info.frameA == "") ? "Unknown" : info.frameA;
        const std::string nameB = (info.frameB == "") ? "Unknown" : info.frameB;
        // Note: setItem takes ownership of the QTableWidgetItems
        if (nameA < nameB) {
            _ui->_framePairTable->setItem (
                row, 0, new QTableWidgetItem (QString::fromStdString (nameA)));
            _ui->_framePairTable->setItem (
                row, 1, new QTableWidgetItem (QString::fromStdString (nameB)));
        }
        else {
            _ui->_framePairTable->setItem (
                row, 0, new QTableWidgetItem (QString::fromStdString (nameB)));
            _ui->_framePairTable->setItem (
                row, 1, new QTableWidgetItem (QString::fromStdString (nameA)));
        }
        _ui->_framePairTable->setItem (
            row, 2, new QTableWidgetItem (QString::number (info.result.distance)));
        row++;
    }
    _ui->_framePairTable->setSortingEnabled (true);
    if (nrOfEntries > 0)
        _ui->_framePairTable->setRangeSelected (
            QTableWidgetSelectionRange (0, 0, nrOfEntries - 1, 2), true);
}

void DistanceMultiResultWidget::showGraphics (const GroupNode::Ptr root,
                                              const SceneGraph::Ptr graph)
{
    if (root == NULL && _root != NULL)
        _root->removeChild ("DistancesMulti");
    _root  = root;
    _graph = graph;
}

std::string DistanceMultiResultWidget::getName () const
{
    return "Multi-Distance Result";
}

DistanceMultiResultWidget::Dispatcher::Dispatcher ()
{}

DistanceMultiResultWidget::Dispatcher::~Dispatcher ()
{}

SimulatorLogEntryWidget*
DistanceMultiResultWidget::Dispatcher::makeWidget (const SimulatorLog::CPtr entry,
                                                   QWidget* parent) const
{
    const rw::common::Ptr< const LogDistanceMultiResult > tentry =
        entry.cast< const LogDistanceMultiResult > ();
    if (!(tentry == NULL))
        return new DistanceMultiResultWidget (tentry, parent);
    RW_THROW ("DistanceMultiResultWidget::Dispatcher (makeWidget): invalid entry!");
    return NULL;
}

bool DistanceMultiResultWidget::Dispatcher::accepts (const SimulatorLog::CPtr entry) const
{
    if (!(entry.cast< const LogDistanceMultiResult > () == NULL))
        return true;
    return false;
}

void DistanceMultiResultWidget::framePairsChanged (const QItemSelection& newSelection,
                                                   const QItemSelection& oldSelection)
{
    using rw::math::Transform3D;

    if (_root == NULL || _dwc == NULL || _positions == NULL)
        return;

    PlainTriMesh<>::Ptr mesh = ownedPtr (new PlainTriMesh<> ());

    const std::vector< LogDistanceMultiResult::ResultInfo >& results = _result->getResults ();
    const QModelIndexList indexes = _ui->_framePairTable->selectionModel ()->selectedIndexes ();

    foreach (const QModelIndex& index, indexes)
    {
        if (index.column () > 0)
            continue;
        const LogDistanceMultiResult::ResultInfo& result =
            results[static_cast< std::size_t > (index.row ())];
        // const Vector3D<> p1 = result.result.p1;
        // const Vector3D<> p2 = result.result.p2;
        for (const Object::Ptr object : _dwc->getWorkcell ()->getObjects ()) {
            for (const Geometry::Ptr geo : object->getGeometry ()) {
                if (geo->getFrame ()->getName () == result.frameA) {
                    for (std::size_t i = 0; i < result.result.geoIdxA.size (); i++) {
                        const int geoIdxA       = result.result.geoIdxA[i];
                        const unsigned int idx1 = result.result.p1prims[i];
                        if (geo->getId () == result.geoNamesA[geoIdxA]) {
                            const TriMesh::Ptr data = geo->getGeometryData ()->getTriMesh ();
                            Transform3D<> T;
                            if (_positions->has (result.frameA))
                                T = _positions->getPosition (result.frameA);
                            T                    = T * geo->getTransform ();
                            const Triangle<> tri = data->getTriangle (idx1).transform (T);
                            mesh->add (tri);
                        }
                    }
                }
                else if (geo->getFrame ()->getName () == result.frameB) {
                    for (std::size_t i = 0; i < result.result.geoIdxB.size (); i++) {
                        const int geoIdxB       = result.result.geoIdxB[i];
                        const unsigned int idx2 = result.result.p2prims[i];
                        if (geo->getId () == result.geoNamesB[geoIdxB]) {
                            const TriMesh::Ptr data = geo->getGeometryData ()->getTriMesh ();
                            Transform3D<> T;
                            if (_positions->has (result.frameB))
                                T = _positions->getPosition (result.frameB);
                            T                    = T * geo->getTransform ();
                            const Triangle<> tri = data->getTriangle (idx2).transform (T);
                            mesh->add (tri);
                        }
                    }
                }
            }
        }
    }

    _root->removeChild ("DistancesMulti");
    GroupNode::Ptr contactGroup      = ownedPtr (new GroupNode ("DistancesMulti"));
    const RenderGeometry::Ptr render = ownedPtr (new RenderGeometry (mesh));
    render->setColor (0, 1, 0);
    const DrawableNode::Ptr drawable =
        _graph->makeDrawable ("DistancesMulti", render, DrawableNode::Virtual);
    GroupNode::addChild (drawable, contactGroup);
    drawable->setVisible (true);

    GroupNode::addChild (contactGroup, _root);
    drawable->setVisible (true);

    emit graphicsUpdated ();
}
