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

#include "LuaEditorWindow.hpp"

#include "CodeEditor.hpp"
#include "LuaExecutionThread.hpp"
#include "LuaHighlighter.hpp"
#include "TreeModelCompleter.hpp"
#include "ui_LuaEditorWindow.h"

#include <rw/core/Log.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/models/Object.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwlibs/swig/lua/LuaState.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QFileDialog>
#include <QStandardItem>
#include <QStringListModel>
#include <QTabWidget>
#include <QTextStream>
#include <boost/filesystem.hpp>

extern "C"
{
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

using namespace rw::core;
using namespace rw::core;
using namespace rw::math;
using namespace rws;
using namespace rwlibs;

namespace {

void luaLineHook (lua_State* L, lua_Debug* ar)
{
    lua_getinfo (L, "nS", ar);
    if (ar->name != NULL && ar->namewhat != NULL) {
        // std::cout << "Name: " << ar->name << " " << ar->namewhat << std::endl;
        // std::cout << "Line: " << ar->currentline << " " << ar->linedefined << std::endl;
    }
}

}    // namespace

LuaEditorWindow::LuaEditorWindow (rwlibs::swig::LuaState::Ptr lua, rw::core::Log::Ptr output,
                                  rws::RobWorkStudio* rwstudio, QWidget* parent) :
    QMainWindow (parent),
    _lua (lua), _output (output), _rws (rwstudio)
{
    _ui = new Ui::LuaEditorWindow ();
    _ui->setupUi (this);

    _luaRunner = new LuaExecutionThread ("", _lua, _output->getWriter (Log::Info), this);
    lua_sethook (_lua->get (), luaLineHook, LUA_MASKLINE, 0);
    _tabPane = new QTabWidget ();

    connect (_luaRunner, SIGNAL (finished ()), this, SLOT (runFinished ()));

    this->setWindowTitle ("Lua Teachpad");
    this->setCentralWidget (_tabPane);
    makeEditor ();
    //  _modified = false;
}

LuaEditorWindow::~LuaEditorWindow ()
{}

void LuaEditorWindow::ShowContextMenu (const QPoint& pos)
{
    QMenu* menu = getCurrentTab ()->_editor->createStandardContextMenu ();
    menu->setTitle ("Edit");
    // for most widgets
    QPoint globalPos = this->mapToGlobal (pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos);

    QMenu myMenu;
    QMenu deviceMenu ("Devices");
    std::vector< rw::models::Device::Ptr > devs = _rws->getWorkcell ()->getDevices ();
    for (rw::models::Device::Ptr dev : devs) {
        QMenu* devMenu = new QMenu (dev->getName ().c_str ());
        connect (devMenu, SIGNAL (triggered (QAction*)), this, SLOT (setCheckAction (QAction*)));
        QAction* action = devMenu->addAction ("Get Q");
        devMenu->addAction (action);
        devMenu->addAction ("Get Pos Limits");
        devMenu->addAction ("Get Vel Limits");
        devMenu->addAction ("Get Acc Limits");

        deviceMenu.addMenu (devMenu);
    }

    QMenu objectMenu ("Objects");
    std::vector< rw::models::Object::Ptr > objects = _rws->getWorkcell ()->getObjects ();
    for (rw::models::Object::Ptr dev : objects) {
        QMenu* devMenu = new QMenu (dev->getName ().c_str ());
        connect (devMenu, SIGNAL (triggered (QAction*)), this, SLOT (setCheckAction (QAction*)));

        QAction* action = devMenu->addAction ("Get Transform");
        devMenu->addAction (action);

        action = devMenu->addAction ("Get Transform W");
        devMenu->addAction (action);

        // devMenu->addAction( "Get Pos Limits" );
        // devMenu->addAction( "Get Vel Limits" );
        // devMenu->addAction( "Get Acc Limits" );

        objectMenu.addMenu (devMenu);
    }

    QMenu* rwsMenu = new QMenu ("RWStudio");
    connect (rwsMenu, SIGNAL (triggered (QAction*)), this, SLOT (setCheckAction (QAction*)));
    QAction* action;
    action = rwsMenu->addAction ("View Transform");
    rwsMenu->addAction (action);
    action = rwsMenu->addAction ("View Transform (RPY)");
    rwsMenu->addAction (action);
    action = rwsMenu->addAction ("View Transform (Quat)");
    rwsMenu->addAction (action);

    myMenu.addMenu (&deviceMenu);
    myMenu.addMenu (&objectMenu);
    myMenu.addMenu (rwsMenu);
    myMenu.addMenu (menu);
    // ...

    QAction* selectedItem = myMenu.exec (globalPos);
    if (selectedItem) {
        // something was chosen, do stuff
    }
    else {
        // nothing was chosen
    }
}

void LuaEditorWindow::setCheckAction (QAction* action)
{
    QObject* obj                = sender ();
    QMenu* menu                 = dynamic_cast< QMenu* > (obj);
    std::string devname         = menu->title ().toStdString ();
    std::string actionStr       = action->text ().toStdString ();
    rw::models::Device::Ptr dev = _rws->getWorkcell ()->findDevice (devname);
    if (actionStr == "Get Q") {
        rw::math::Q q = dev->getQ (_rws->getState ());
        std::stringstream sstr;
        sstr << "rw.Q(" << q.size () << ",{";
        for (size_t i = 0; i < q.size () - 1; i++)
            sstr << q[i] << ",";
        sstr << q[q.size () - 1] << "})";
        getCurrentTab ()->_editor->insertCompletion (sstr.str ().c_str ());
    }
    else if (actionStr == "View Transform (RPY)") {
        rw::math::Transform3D<> t3d = _rws->getView ()->getSceneViewer ()->getTransform ();
        RPY<> rpy (t3d.R ());
        std::stringstream sstr;
        sstr << "rw.Transform3D( rw.Vector3D(" << t3d.P ()[0] << "," << t3d.P ()[1] << ","
             << t3d.P ()[2] << "),"
             << "rw.RPY(" << rpy[0] << "," << rpy[1] << "," << rpy[2] << "))";
        getCurrentTab ()->_editor->insertCompletion (sstr.str ().c_str ());
    }
    // std::cout << menu->title().toStdString() << std::endl;
    // std::cout <<  << std::endl;
}

LuaEditorWindow::EditorTab::Ptr LuaEditorWindow::makeEditor ()
{
    QFont font;
    font.setFamily ("Courier");
    font.setFixedPitch (true);
    font.setPointSize (10);

    EditorTab::Ptr etab = ownedPtr (new EditorTab ());
    etab->_editor       = new CodeEditor (this);
    etab->_editor->setFont (font);

    const int tabStop = 4;    // 4 characters
    QFontMetrics metrics (font);
#if QT_VERSION >= QT_VERSION_CHECK(5, 10, 0)
#if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
    etab->_editor->setTabStopDistance (tabStop * metrics.horizontalAdvance (' '));
#else
    etab->_editor->setTabStopDistance (tabStop * metrics.width (' '));
#endif
#else
    etab->_editor->setTabStopWidth (tabStop * metrics.width (' '));
#endif

    etab->_completer = new TreeModelCompleter (etab->_editor);
    etab->_completer->setSeparator (QLatin1String ("."));
    etab->_completer->setModel (modelFromFile (":/wordlist.txt", etab->_completer));
    etab->_completer->setModelSorting (QCompleter::CaseInsensitivelySortedModel);
    etab->_completer->setCaseSensitivity (Qt::CaseInsensitive);
    etab->_completer->setWrapAround (false);
    etab->_editor->setCompleter (etab->_completer);
    etab->_highlighter = new LuaHighlighter (etab->_editor->document ());
    connect (etab->_editor, SIGNAL (modificationChanged (bool)), this, SLOT (textChanged ()));
    etab->_editor->setContextMenuPolicy (Qt::CustomContextMenu);
    connect (etab->_editor,
             SIGNAL (customContextMenuRequested (const QPoint&)),
             this,
             SLOT (ShowContextMenu (const QPoint&)));
    _tabPane->addTab (etab->_editor, "New");
    _editors[etab->_editor] = etab;
    return etab;
}

void LuaEditorWindow::runFinished ()
{
    // std::cout << "FINISHED" << std::endl;
    _tabPane->setEnabled (true);
    //_editor->setEnabled(true);
}

void LuaEditorWindow::textChanged ()
{
    //_modified = _editor->document()->isModified();
}

void LuaEditorWindow::on_actionClose_triggered (bool)
{
    // todo: if last tab then don't remove it.
    if (_tabPane->count () == 1)
        return;
    EditorTab::Ptr tab = getCurrentTab ();
    if (tab->_editor->document ()->isModified ())
        save ();
    _tabPane->removeTab (_tabPane->indexOf (tab->_editor));
    _editors[tab->_editor] = NULL;
}

void LuaEditorWindow::on_actionNew_triggered (bool)
{
    // create a new tab
    EditorTab::Ptr tab = makeEditor ();
    _tabPane->setCurrentIndex (_tabPane->indexOf (tab->_editor));
    tab->_editor->document ()->setModified (false);

    /*
            if (_editor->document()->isModified()) {
        //if (_modified) {
            int result = QMessageBox::warning(this, "Lua Editor", tr("Content has been modified. Do
       you wish to save changes?"), QMessageBox::Yes, QMessageBox::No, QMessageBox::Cancel); switch
       (result) { case QMessageBox::Yes: if (!save()) return; break; case QMessageBox::No: break;
                case QMessageBox::Cancel:
                    return;
            }
        }
        _editor->clear();
        _editor->document()->setModified(false);
     */
}

void LuaEditorWindow::on_actionOpen_triggered (bool)
{
    // on_actionNew_triggered(true);
    QString path = _pmap.get< std::string > ("PreviousOpenDirectory", ".").c_str ();

    QString fileName = QFileDialog::getOpenFileName (this,
                                                     tr ("Open File"),
                                                     path,
                                                     "Supported (*.lua *.txt)"
                                                     "\nLua Files (*.lua)"
                                                     "\nAll (*.*)");

    if (!fileName.isEmpty ()) {
        _pmap.set< std::string > ("PreviousOpenDirectory",
                                  StringUtil::getDirectoryName (fileName.toStdString ()));
        _pmap.set< std::string > ("LuaFile", fileName.toStdString ());
        QFile file;
        file.setFileName (fileName);
        EditorTab::Ptr tab = makeEditor ();
        tab->_filename     = file.fileName ().toStdString ();
        _tabPane->setTabText (
            _tabPane->indexOf (tab->_editor),
            boost::filesystem::path (fileName.toStdString ()).filename ().string ().c_str ());

        if (file.open (QFile::ReadWrite | QFile::Text)) {
            tab->_editor->setPlainText (file.readAll ());
        }
        file.close ();
        tab->_editor->document ()->setModified (false);
        _tabPane->setCurrentIndex (_tabPane->indexOf (tab->_editor));
    }
}

void LuaEditorWindow::on_actionSave_triggered (bool)
{
    save ();
}

void LuaEditorWindow::on_actionSave_As_triggered (bool)
{
    saveAs ();
}

bool LuaEditorWindow::save ()
{
    // save the current viable
    std::string filename = getCurrentTab ()->_filename;
    // std::string filename = _pmap.get<std::string>("LuaFile","");
    if (filename == "") {
        return saveAs ();
    }
    else {
        return save (filename);
    }
}

bool LuaEditorWindow::saveAs ()
{
    std::string defaultName = _pmap.get< std::string > ("LuaFile", "");
    QString filename        = QFileDialog::getSaveFileName (
        this, "Save as", defaultName.c_str (), "Supported (*.lua *.txt)");

    if (filename.isEmpty ())
        return false;

    return save (filename.toStdString ());
}

bool LuaEditorWindow::save (const std::string& filename)
{
    QFile file;
    file.setFileName (filename.c_str ());
    if (file.open (QFile::WriteOnly | QFile::Text)) {
        {
            QTextStream out (&file);
            out << getCurrentTab ()->_editor->toPlainText ();
            _pmap.set< std::string > ("LuaFile", filename);
            getCurrentTab ()->_editor->document ()->setModified (false);
            getCurrentTab ()->_filename = filename;

            _tabPane->setTabText (
                _tabPane->indexOf (getCurrentTab ()->_editor),
                boost::filesystem::path (filename).filename ().string ().c_str ());
        }
        file.close ();
        return true;
    }
    else {
        file.close ();
        return false;
    }
}

LuaEditorWindow::EditorTab::Ptr LuaEditorWindow::getCurrentTab ()
{
    return _editors[_tabPane->currentWidget ()];
}

void LuaEditorWindow::on_actionRun_triggered (bool)
{
    //_lua->reset();
    rwlibs::swig::setlog (&Log::infoLog ());
    const std::string cmd = getCurrentTab ()->_editor->toPlainText ().toStdString ();
    // getCurrentTab()->_editor->setEnabled(false);
    _luaRunner->set (cmd, _lua, _output->getWriter (Log::Info));
    _luaRunner->start ();
    //_luaRunner->run(); // TODO: some lua code will not work in separate threads
}

void LuaEditorWindow::on_actionReset_triggered (bool)
{
    _lua->reset ();
    rwlibs::swig::setlog (&Log::infoLog ());
}

void LuaEditorWindow::on_actionReload_triggered (bool)
{
    std::string fileName = getCurrentTab ()->_filename;
    if (fileName == "")
        return;

    QFile file;
    file.setFileName (fileName.c_str ());
    if (file.open (QFile::ReadWrite | QFile::Text)) {
        getCurrentTab ()->_editor->setPlainText (file.readAll ());
    }
    file.close ();
}

void LuaEditorWindow::on_actionStop_triggered (bool)
{
    _luaRunner->stop ();
}

QAbstractItemModel* LuaEditorWindow::modelFromFile (const QString& fileName,
                                                    TreeModelCompleter* completer)
{
    QFile file (fileName);
    if (!file.open (QFile::ReadOnly))
        return new QStringListModel (completer);

#ifndef QT_NO_CURSOR
    QApplication::setOverrideCursor (QCursor (Qt::WaitCursor));
#endif
    QStringList words;

    QStandardItemModel* model = new QStandardItemModel (completer);
    QVector< QStandardItem* > parents (10);
    parents[0] = model->invisibleRootItem ();

    while (!file.atEnd ()) {
        QString line        = file.readLine ();
        QString trimmedLine = line.trimmed ();
        if (line.isEmpty () || trimmedLine.isEmpty ())
            continue;

        QRegularExpression re ("^\\s+");
        const QRegularExpressionMatch match = re.match (line);
        int level                           = 0;
        if (!match.hasMatch ()) {
            level = 0;
        }
        else {
            if (line.startsWith ("\t")) {
                level = match.capturedLength (0);
            }
            else {
                level = match.capturedLength (0) / 4;
            }
        }

        if (level + 1 >= parents.size ())
            parents.resize (parents.size () * 2);

        QStandardItem* item = new QStandardItem;
        item->setText (trimmedLine);
        parents[level]->appendRow (item);
        parents[level + 1] = item;
    }

#ifndef QT_NO_CURSOR
    QApplication::restoreOverrideCursor ();
#endif

    return model;
}
