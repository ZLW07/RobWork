/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ContactVelocitiesWidget.hpp"

#include "ui_ContactVelocitiesWidget.h"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderVelocity.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogContactSet.hpp>
#include <rwsim/log/LogContactVelocities.hpp>

#include <QItemSelection>

using namespace rw::core;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

ContactVelocitiesWidget::ContactVelocitiesWidget (rw::core::Ptr< const LogContactVelocities > entry,
                                                  QWidget* parent) :
    SimulatorLogEntryWidget (parent),
    _ui (new Ui::ContactVelocitiesWidget ()), _velocities (entry),
    _contactSet (entry->getContacts ())
{
    _ui->setupUi (this);
    connect (_ui->_contactBodyPairs->selectionModel (),
             SIGNAL (selectionChanged (const QItemSelection&, const QItemSelection&)),
             this,
             SLOT (contactSetPairsChanged (const QItemSelection&, const QItemSelection&)));
    connect (_ui->_contactTable->selectionModel (),
             SIGNAL (selectionChanged (const QItemSelection&, const QItemSelection&)),
             this,
             SLOT (contactSetChanged (const QItemSelection&, const QItemSelection&)));

    QStringList headerLabels;
    headerLabels.push_back ("First");
    headerLabels.push_back ("Second");
    headerLabels.push_back ("Contacts");
    _ui->_contactBodyPairs->setHorizontalHeaderLabels (headerLabels);

    _ui->_contactTable->setColumnCount (8);
    headerLabels.clear ();
    headerLabels.push_back ("First");
    headerLabels.push_back ("x");
    headerLabels.push_back ("y");
    headerLabels.push_back ("z");
    headerLabels.push_back ("Second");
    headerLabels.push_back ("x");
    headerLabels.push_back ("y");
    headerLabels.push_back ("z");
    _ui->_contactTable->setHorizontalHeaderLabels (headerLabels);
}

ContactVelocitiesWidget::~ContactVelocitiesWidget ()
{
    if (_root != NULL) {
        _root->removeChild ("ContactVelocities");
    }
}

void ContactVelocitiesWidget::setDWC (rw::core::Ptr< const DynamicWorkCell > dwc)
{}

void ContactVelocitiesWidget::setEntry (rw::core::Ptr< const SimulatorLog > entry)
{
    const rw::core::Ptr< const LogContactVelocities > set =
        entry.cast< const LogContactVelocities > ();
    if (!(set == NULL)) {
        _velocities = set;
        _contactSet = _velocities->getContacts ();
    }
    else {
        RW_THROW ("ContactVelocitiesWidget (setEntry): invalid entry!");
    }
}

rw::core::Ptr< const SimulatorLog > ContactVelocitiesWidget::getEntry () const
{
    return _contactSet;
}

void ContactVelocitiesWidget::updateEntryWidget ()
{
    _ui->_contactDescription->setText (QString::fromStdString (_velocities->getDescription ()));
    _ui->_contacts->setText (QString::number (_contactSet->size ()));
    typedef std::pair< std::string, std::string > FramePair;
    std::set< FramePair > pairs;
    for (const Contact& c : _contactSet->getContacts ()) {
        const std::string& nameA = c.getNameA ();
        const std::string& nameB = c.getNameB ();
        if (nameA < nameB)
            pairs.insert (std::make_pair (nameA, nameB));
        else
            pairs.insert (std::make_pair (nameB, nameA));
    }
    const int nrOfPairs = static_cast< int > (pairs.size ());
    if (pairs.size () > static_cast< std::size_t > (nrOfPairs))
        RW_THROW ("There are too many entries for the widget to handle!");
    _ui->_contactBodyPairs->setRowCount (nrOfPairs);
    int row = 0;
    _ui->_contactBodyPairs->setSortingEnabled (false);
    for (const FramePair& pair : pairs) {
        // Count how many contacts there are for this pair
        int contacts = 0;
        for (const Contact& c : _contactSet->getContacts ()) {
            if (c.getNameA () == pair.first && c.getNameB () == pair.second)
                contacts++;
            else if (c.getNameA () == pair.second && c.getNameB () == pair.first)
                contacts++;
        }
        // Note: setItem takes ownership of the QTableWidgetItems
        _ui->_contactBodyPairs->setItem (
            row, 0, new QTableWidgetItem (QString::fromStdString (pair.first)));
        _ui->_contactBodyPairs->setItem (
            row, 1, new QTableWidgetItem (QString::fromStdString (pair.second)));
        _ui->_contactBodyPairs->setItem (row, 2, new QTableWidgetItem (QString::number (contacts)));
        row++;
    }
    _ui->_contactBodyPairs->setSortingEnabled (true);
    if (nrOfPairs > 0)
        _ui->_contactBodyPairs->setRangeSelected (
            QTableWidgetSelectionRange (0, 0, nrOfPairs - 1, 2), true);
}

void ContactVelocitiesWidget::showGraphics (GroupNode::Ptr root, SceneGraph::Ptr graph)
{
    if (root == NULL && _root != NULL)
        _root->removeChild ("Bodies");
    _root  = root;
    _graph = graph;
}

std::string ContactVelocitiesWidget::getName () const
{
    return "Contact Velocities";
}

void ContactVelocitiesWidget::contactSetPairsChanged (const QItemSelection&, const QItemSelection&)
{
    typedef std::pair< std::string, std::string > NamePair;
    const QModelIndexList indexes = _ui->_contactBodyPairs->selectionModel ()->selectedIndexes ();
    std::list< NamePair > names;
    foreach (QModelIndex index, indexes)
    {
        if (index.column () != 0)
            continue;
        const std::string nameA = _ui->_contactBodyPairs->item (index.row (), 0)
                                      ->data (Qt::DisplayRole)
                                      .toString ()
                                      .toStdString ();
        const std::string nameB = _ui->_contactBodyPairs->item (index.row (), 1)
                                      ->data (Qt::DisplayRole)
                                      .toString ()
                                      .toStdString ();
        names.push_back (NamePair (nameA, nameB));
    }
    std::vector< std::size_t > contactsToShow;
    for (std::size_t i = 0; i < _contactSet->size (); i++) {
        const Contact& c = _contactSet->getContact (i);
        bool show        = false;
        for (const NamePair& name : names) {
            const std::string& nameA = c.getNameA ();
            const std::string& nameB = c.getNameB ();
            if (nameA == name.first && nameB == name.second)
                show = true;
            else if (nameB == name.first && nameA == name.second)
                show = true;
            if (show)
                break;
        }
        if (show)
            contactsToShow.push_back (i);
    }
    const int nrOfContactsToShow = static_cast< int > (contactsToShow.size ());
    if (contactsToShow.size () > static_cast< std::size_t > (nrOfContactsToShow))
        RW_THROW ("There are too many entries for the widget to handle!");
    _ui->_contactTable->clearSelection ();
    _ui->_contactTable->setRowCount (nrOfContactsToShow);
    int row = 0;
    _ui->_contactTable->setSortingEnabled (false);
    for (const std::size_t i : contactsToShow) {
        const Contact& c         = _contactSet->getContact (i);
        const Vector3D<> velA    = _velocities->getVelocityBodyA (i);
        const Vector3D<> velB    = _velocities->getVelocityBodyB (i);
        const std::string& nameA = c.getNameA ();
        const std::string& nameB = c.getNameB ();
        QString hover            = toQString (c, velA, velB);
        // Note: setItem takes ownership of the QTableWidgetItems
        QTableWidgetItem *itemA, *itemB;
        QTableWidgetItem *itemAx, *itemAy, *itemAz;
        QTableWidgetItem *itemBx, *itemBy, *itemBz;
        if (nameA < nameB) {
            itemA  = new QTableWidgetItem (QString::fromStdString (nameA));
            itemAx = new QTableWidgetItem (QString::number (velA[0]));
            itemAy = new QTableWidgetItem (QString::number (velA[1]));
            itemAz = new QTableWidgetItem (QString::number (velA[2]));
            itemB  = new QTableWidgetItem (QString::fromStdString (nameB));
            itemBx = new QTableWidgetItem (QString::number (velB[0]));
            itemBy = new QTableWidgetItem (QString::number (velB[1]));
            itemBz = new QTableWidgetItem (QString::number (velB[2]));
        }
        else {
            itemA  = new QTableWidgetItem (QString::fromStdString (nameB));
            itemAx = new QTableWidgetItem (QString::number (velB[0]));
            itemAy = new QTableWidgetItem (QString::number (velB[1]));
            itemAz = new QTableWidgetItem (QString::number (velB[2]));
            itemB  = new QTableWidgetItem (QString::fromStdString (nameA));
            itemBx = new QTableWidgetItem (QString::number (velA[0]));
            itemBy = new QTableWidgetItem (QString::number (velA[1]));
            itemBz = new QTableWidgetItem (QString::number (velA[2]));
        }
        if (dot (velB - velA, c.getNormal ()) < 0) {
            itemA->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemAx->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemAy->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemAz->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemB->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemBx->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemBy->setData (Qt::ForegroundRole, QColor (Qt::red));
            itemBz->setData (Qt::ForegroundRole, QColor (Qt::red));
            hover = hover + "\nPenetration velocity: " +
                    QString::number (-dot (velB - velA, c.getNormal ()));
        }
        else {
            itemA->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemAx->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemAy->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemAz->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemB->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemBx->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemBy->setData (Qt::ForegroundRole, QColor (Qt::green));
            itemBz->setData (Qt::ForegroundRole, QColor (Qt::green));
            hover = hover +
                    "\nLeaving velocity: " + QString::number (dot (velB - velA, c.getNormal ()));
        }
        itemA->setData (Qt::ToolTipRole, hover);
        itemB->setData (Qt::ToolTipRole, hover);
        itemAx->setData (Qt::ToolTipRole, hover);
        itemAy->setData (Qt::ToolTipRole, hover);
        itemAz->setData (Qt::ToolTipRole, hover);
        itemBx->setData (Qt::ToolTipRole, hover);
        itemBy->setData (Qt::ToolTipRole, hover);
        itemBz->setData (Qt::ToolTipRole, hover);
        itemA->setData (Qt::UserRole, QVariant::fromValue (i));
        _ui->_contactTable->setItem (row, 0, itemA);
        _ui->_contactTable->setItem (row, 1, itemAx);
        _ui->_contactTable->setItem (row, 2, itemAy);
        _ui->_contactTable->setItem (row, 3, itemAz);
        _ui->_contactTable->setItem (row, 4, itemB);
        _ui->_contactTable->setItem (row, 5, itemBx);
        _ui->_contactTable->setItem (row, 6, itemBy);
        _ui->_contactTable->setItem (row, 7, itemBz);
        row++;
    }
    _ui->_contactTable->setSortingEnabled (true);
    if (nrOfContactsToShow > 0)
        _ui->_contactTable->setRangeSelected (
            QTableWidgetSelectionRange (0, 0, nrOfContactsToShow - 1, 7), true);
}

void ContactVelocitiesWidget::contactSetChanged (const QItemSelection&, const QItemSelection&)
{
    const QModelIndexList indexes = _ui->_contactTable->selectionModel ()->selectedIndexes ();
    _root->removeChild ("ContactVelocities");
    GroupNode::Ptr contactGroup = ownedPtr (new GroupNode ("ContactVelocities"));
    foreach (QModelIndex index, indexes)
    {
        if (index.column () > 0)
            continue;
        const std::size_t i = index.data (Qt::UserRole).toUInt ();
        // contacts.push_back(_contactSet->contacts[i]);
        const RenderVelocity::Ptr renderA = ownedPtr (new RenderVelocity ());
        const RenderVelocity::Ptr renderB = ownedPtr (new RenderVelocity ());
        renderA->setVelocity (VelocityScrew6D<> (_velocities->getVelocityBodyA (i), EAA<> ()));
        renderB->setVelocity (VelocityScrew6D<> (_velocities->getVelocityBodyB (i), EAA<> ()));
        const DrawableNode::Ptr drawableA =
            _graph->makeDrawable ("ContactVelocityA", renderA, DrawableNode::Physical);
        const DrawableNode::Ptr drawableB =
            _graph->makeDrawable ("ContactVelocityB", renderB, DrawableNode::Physical);
        drawableA->setTransform (
            Transform3D<> (_velocities->getContacts ()->getContact (i).getPointA ()));
        drawableB->setTransform (
            Transform3D<> (_velocities->getContacts ()->getContact (i).getPointB ()));
        GroupNode::addChild (drawableA, contactGroup);
        GroupNode::addChild (drawableB, contactGroup);
        drawableA->setVisible (true);
        drawableB->setVisible (true);
    }
    GroupNode::addChild (contactGroup, _root);
    emit graphicsUpdated ();
}

QString ContactVelocitiesWidget::toQString (const Vector3D<>& vec)
{
    std::stringstream str;
    str << vec;
    return QString::fromStdString (str.str ());
}

QString ContactVelocitiesWidget::toQString (const Contact& contact, const Vector3D<>& velA,
                                            const Vector3D<>& velB)
{
    const QString nameA = QString::fromStdString (contact.getNameA ());
    const QString nameB = QString::fromStdString (contact.getNameB ());
    return nameA + ": " + toQString (velA) + "<br/>" + nameB + ": " + toQString (velB);
}

ContactVelocitiesWidget::Dispatcher::Dispatcher ()
{}

ContactVelocitiesWidget::Dispatcher::~Dispatcher ()
{}

SimulatorLogEntryWidget*
ContactVelocitiesWidget::Dispatcher::makeWidget (rw::core::Ptr< const SimulatorLog > entry,
                                                 QWidget* parent) const
{
    const rw::core::Ptr< const LogContactVelocities > velocities =
        entry.cast< const LogContactVelocities > ();
    if (!(velocities == NULL))
        return new ContactVelocitiesWidget (velocities, parent);
    RW_THROW ("ContactVelocitiesWidget::Dispatcher (makeWidget): invalid entry!");
    return NULL;
}

bool ContactVelocitiesWidget::Dispatcher::accepts (rw::core::Ptr< const SimulatorLog > entry) const
{
    if (!(entry.cast< const LogContactVelocities > () == NULL))
        return true;
    return false;
}
