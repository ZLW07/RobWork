/********************************************************************************
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
 ********************************************************************************/

#include "WorkcellEditorWindow.hpp"

#include "InputFormDialog.hpp"
#include "WCCodeEditor.hpp"
#include "WCECompleter.hpp"
#include "WorkcellHighlighter.hpp"
#include "ui_WorkcellEditorWindow.h"

#include <rw/core/Log.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QColor>
#include <QDebug>
#include <QFileDialog>
#include <QPushButton>
#include <QStandardItem>
#include <QStringListModel>
#include <QTabWidget>
#include <QTextStream>
#include <QVector2D>
#include <QVector3D>
#include <boost/filesystem.hpp>
#include <fstream>

// using namespace xercesc;
using namespace rw::core;
using namespace rw::loaders;
using namespace rw::math;
using namespace rws;
using namespace rwlibs;

WorkcellEditorWindow::WorkcellEditorWindow (rw::core::Log::Ptr output, rws::RobWorkStudio* rwstudio,
                                            QWidget* parent) :
    QMainWindow (parent),
    _output (output), _rws (rwstudio), ignoreNextWorkcellOpen (false)
{
    _ui = new Ui::WorkcellEditorWindow ();
    _ui->setupUi (this);
    _tabPane = new QTabWidget ();
    _tabPane->setTabsClosable (true);
    connect (
        _tabPane, SIGNAL (tabCloseRequested (int)), this, SLOT (on_actionClose_triggered (int)));

    this->setWindowTitle ("Workcell Editor");
    this->setCentralWidget (_tabPane);
    makeEditor ();

    openWorkCell (_rws->getWorkcell ()->getFilePath ().c_str ());
    rw::core::Log::infoLog () << "Opening: workcell "
                              << _rws->getWorkcell ()->getFilePath ().c_str () << "\n";
}

WorkcellEditorWindow::~WorkcellEditorWindow ()
{}

void WorkcellEditorWindow::ShowContextMenu (const QPoint& pos)
{
    /*QMenu *menu = getCurrentTab()->_editor->createStandardContextMenu();
    menu->setTitle("Edit");
    // for most widgets
    QPoint globalPos = this->mapToGlobal(pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos);

    QMenu myMenu;
    QMenu deviceMenu("Devices");
    std::vector<rw::models::Device::Ptr> devs = _rws->getWorkcell()->getDevices();
    for (rw::models::Device::Ptr dev : devs) {
        QMenu *devMenu = new QMenu(dev->getName().c_str());
        connect(devMenu, SIGNAL(triggered(QAction * )), this, SLOT(setCheckAction(QAction * )));
        QAction *action = devMenu->addAction("Get Q");
        devMenu->addAction(action);
        devMenu->addAction("Get Pos Limits");
        devMenu->addAction("Get Vel Limits");
        devMenu->addAction("Get Acc Limits");

        deviceMenu.addMenu(devMenu);
    }

    QMenu objectMenu("Objects");
    std::vector<rw::models::Object::Ptr> objects = _rws->getWorkcell()->getObjects();
    for (rw::models::Object::Ptr dev : objects) {
        QMenu *devMenu = new QMenu(dev->getName().c_str());
        connect(devMenu, SIGNAL(triggered(QAction * )), this, SLOT(setCheckAction(QAction * )));

        QAction *action = devMenu->addAction("Get Transform");
        devMenu->addAction(action);

        action = devMenu->addAction("Get Transform W");
        devMenu->addAction(action);


        //devMenu->addAction( "Get Pos Limits" );
        //devMenu->addAction( "Get Vel Limits" );
        //devMenu->addAction( "Get Acc Limits" );

        objectMenu.addMenu(devMenu);
    }

    QMenu *rwsMenu = new QMenu("RWStudio");
    connect(rwsMenu, SIGNAL(triggered(QAction * )), this, SLOT(setCheckAction(QAction * )));
    QAction *action;
    action = rwsMenu->addAction("View Transform");
    rwsMenu->addAction(action);
    action = rwsMenu->addAction("View Transform (RPY)");
    rwsMenu->addAction(action);
    action = rwsMenu->addAction("View Transform (Quat)");
    rwsMenu->addAction(action);


    myMenu.addMenu(&deviceMenu);
    myMenu.addMenu(&objectMenu);
    myMenu.addMenu(rwsMenu);
    myMenu.addMenu(menu);
    // ...

    QAction *selectedItem = myMenu.exec(globalPos);
    if (selectedItem) {
        // something was chosen, do stuff
    } else {
        // nothing was chosen
    }*/
}

void WorkcellEditorWindow::setCheckAction (QAction* action)
{
    /*QObject *obj = sender();
    QMenu *menu = dynamic_cast<QMenu *>(obj);
    std::string devname = menu->title().toStdString();
    std::string actionStr = action->text().toStdString();
    rw::models::Device::Ptr dev = _rws->getWorkcell()->findDevice(devname);
    if (actionStr == "Get Q") {
        rw::math::Q q = dev->getQ(_rws->getState());
        std::stringstream sstr;
        sstr << "rw.Q(" << q.size() << ",{";
        for (size_t i = 0; i < q.size() - 1; i++)
            sstr << q[i] << ",";
        sstr << q[q.size() - 1] << "})";
        getCurrentTab()->_editor->insertCompletion(sstr.str().c_str());
    } else if (actionStr == "View Transform (RPY)") {
        rw::math::Transform3D<> t3d =
            _rws->getView()->getSceneViewer()->getTransform(); RPY<> rpy(t3d.R());
        std::stringstream sstr;
        sstr << "rw.Transform3D( rw.Vector3D(" << t3d.P()[0] << "," <<
            t3d.P()[1] << "," << t3d.P()[2] << "),"
             << "rw.RPY(" << rpy[0] << "," << rpy[1] << "," << rpy[2] << "))";
        getCurrentTab()->_editor->insertCompletion(sstr.str().c_str());
    }*/
}

WorkcellEditorWindow::EditorTab::Ptr WorkcellEditorWindow::makeEditor ()
{
    QFont font;
    font.setFamily ("Courier");
    font.setFixedPitch (true);
    font.setPointSize (12);

    EditorTab::Ptr etab = ownedPtr (new EditorTab ());
    etab->_editor       = new WCCodeEditor (this);
    etab->_editor->setFont (font);

    QPalette p            = etab->_editor->palette ();
    QColor* editorBgColor = new QColor (238, 238, 238);
    p.setColor (QPalette::Active, QPalette::Base, editorBgColor->rgb ());
    p.setColor (QPalette::Inactive, QPalette::Base, editorBgColor->rgb ());
    etab->_editor->setPalette (p);

    const int tabStop = 2;    // 2 characters
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

    etab->_completer = new WCECompleter (etab->_editor);
    // etab->_completer->setSeparator (QLatin1String ("."));
    etab->_completer->setModel (modelFromFile (":/word_list.txt", etab->_completer));
    etab->_completer->setCompletionMode (QCompleter::PopupCompletion);
    etab->_completer->setModelSorting (QCompleter::CaseInsensitivelySortedModel);
    etab->_completer->setCaseSensitivity (Qt::CaseInsensitive);
    etab->_completer->setWrapAround (false);

    etab->_editor->setCompleter (etab->_completer);

    etab->_highlighter = new WorkcellHighlighter (etab->_editor->document ());
    // connect(etab->_editor, SIGNAL(modificationChanged(bool)), this,
    // SLOT(textChanged()));
    // etab->_editor->setContextMenuPolicy(Qt::CustomContextMenu);
    // connect(etab->_editor, SIGNAL(customContextMenuRequested(const QPoint&)),
    //		this, SLOT(ShowContextMenu(const QPoint&)));

    // Add workcell tag to new workcells by default with empty name attribute
    QString default_text = "<WorkCell name=\"\">\n\n</WorkCell>";
    etab->_editor->setPlainText (default_text);
    _tabPane->addTab (etab->_editor, "New");
    _editors[etab->_editor] = etab;
    return etab;
}

void WorkcellEditorWindow::runFinished ()
{
    _tabPane->setEnabled (true);
}

void WorkcellEditorWindow::textChanged ()
{
    //_modified = _editor->document()->isModified();
}

void WorkcellEditorWindow::on_actionClose_triggered (bool)
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

void WorkcellEditorWindow::on_actionClose_triggered (int index)
{
    // todo: if last tab then don't remove it.
    if (_tabPane->count () == 1)
        return;
    EditorTab::Ptr tab = _editors[_tabPane->widget (index)];
    if (tab->_editor->document ()->isModified ())
        save ();
    _tabPane->removeTab (index);
    _editors[tab->_editor] = NULL;
}

void WorkcellEditorWindow::on_actionNew_triggered (bool)
{
    // create a new tab
    EditorTab::Ptr tab = makeEditor ();
    _tabPane->setCurrentIndex (_tabPane->indexOf (tab->_editor));
    QString default_text = "<WorkCell name=\"\">\n\n</WorkCell>";
    tab->_editor->setPlainText (default_text);
    tab->_editor->document ()->setModified (false);
}

void WorkcellEditorWindow::on_actionOpen_triggered (bool)
{
    // on_actionNew_triggered(true);
    QString path = _pmap.get< std::string > ("PreviousOpenDirectory", ".").c_str ();

    QString fileName = QFileDialog::getOpenFileName (this,
                                                     tr ("Open File"),
                                                     path,
                                                     "Supported (*.wc.xml *.xml)"
                                                     "\nWorkcell Files (*.wc.xml)"
                                                     "\nXML Files (*.xml)"
                                                     "\nAll (*.*)");

    openWorkCell (fileName);
}

void WorkcellEditorWindow::on_actionSave_triggered (bool)
{
    save ();
}

void WorkcellEditorWindow::on_actionSave_As_triggered (bool)
{
    saveAs ();
}

void WorkcellEditorWindow::on_actionAdd_Frame_triggered (bool)
{
    // Load Workcell

    // Define form inputs
    InputFormDialog::FormData data;
    data["Name"]          = "";
    QStringList refFrames = getRefFrameList ();
    if (refFrames.isEmpty ()) {
        std::cout << "wc is null" << std::endl;
        data["Ref. frame"] = "";
    }
    else {
        data["Ref. frame"] = refFrames;
    }

    data["Type"] = QStringList () << "Fixed"
                                  << "Movable";
    data["Show Frame Axis"] = false;
    data["Position"]        = QVector3D (0.0, 0.0, 0.0);
    data["RPY"]             = QVector3D (0.0, 0.0, 0.0);

    /*data["Bool"] = true;
    data["Color"] = QColor("red");
    data["Int"] = 1;
    data["String"] = "Test";
    data["ComboBox"] = QStringList() << "One" << "Two";
    data["Vector2"] = QVector2D(10.0, 5.0);*/

    // Define form options
    InputFormDialog::FormOptions options;
    options.listDisplaysAsRadios = false;
    options.listReturnsIndex     = false;

    // Ask user for input and retrieve data
    if (InputFormDialog::getInput ("Add Frame", data, options)) {
        // qDebug() << data.at<QString>("Name");
        // qDebug() << data.at<QString>("Ref. frame");
        // qDebug() << data.at<QString>("Type");
        // qDebug() << data.at<QVector3D>("Position");
        // qDebug() << data.at<QVector3D>("RPY");
        QString frame_type = "";
        if (data.at< QString > ("Type") == "Movable")
            frame_type = "type=\"Movable\"";

        QString show_frame_axis = "";
        if (data.at< bool > ("Show Frame Axis") == true) {
            show_frame_axis = "\t<Property name=\"ShowFrameAxis\">true</Property>\n";
        }

        QString frame_str;
        QTextStream (&frame_str) << "<Frame name=\"" << data.at< QString > ("Name") << "\" "
                                 << "refframe=\"" << data.at< QString > ("Ref. frame") << "\" "
                                 << frame_type << ">"
                                 << "\n\t"
                                 << "<Pos>" << data.at< QVector3D > ("Position").x () << " "
                                 << data.at< QVector3D > ("Position").y () << " "
                                 << data.at< QVector3D > ("Position").z () << "</Pos>\n\t"
                                 << "<RPY>" << data.at< QVector3D > ("RPY").x () << " "
                                 << data.at< QVector3D > ("RPY").y () << " "
                                 << data.at< QVector3D > ("RPY").z () << "</RPY>"
                                 << "\n"
                                 << show_frame_axis << "</Frame>";

        // Insert frame at cursor position
        getCurrentTab ()->_editor->insertXMLTextUnderCursor (frame_str);
    }
}

void WorkcellEditorWindow::on_actionAdd_Drawable_triggered (bool)
{
    std::string defaultName = _pmap.get< std::string > ("WorkcellFile", "");
    std::size_t found       = defaultName.find_last_of ("/\\");
    std::string cad_path    = defaultName.substr (0, found);

    // Define form options
    InputFormDialog::FormOptions options;
    options.listDisplaysAsRadios = false;
    options.listReturnsIndex     = false;

    // Define form inputs
    InputFormDialog::FormData data;
    data["Name"]          = "";
    QStringList refFrames = getRefFrameList ();
    if (refFrames.isEmpty ()) {
        std::cout << "wc is null" << std::endl;
        data["Ref. frame"] = "";
    }
    else {
        data["Ref. frame"] = refFrames;
    }
    data["CollisionModel"]  = true;
    data["Mesh Resolution"] = 20;
    data["Position"]        = QVector3D (0.0, 0.0, 0.0);
    data["RPY"]             = QVector3D (0.0, 0.0, 0.0);
    data["CAD file"]        = "cad_file," + QString::fromStdString (cad_path);
    data["Geometry Type"]   = QStringList () << "Box"
                                           << "Cone"
                                           << "Cylinder"
                                           << "Sphere";
    data["Color (r,g,b)"]             = QVector3D (0.6, 0.6, 0.6);
    data["Box (x, y, z)"]             = QVector3D (0.0, 0.0, 0.0);
    data["Cone (radiusBot, height)"]  = QVector2D (0.0, 0.0);
    data["Cylinder (radius, height)"] = QVector2D (0.0, 0.0);
    data["Sphere (r)"]                = 0.0;

    // Ask user for input and retrieve data

    if (InputFormDialog::getInput ("Add Drawable", data, options)) {
        QString col_model = "Enabled";
        if (data.at< bool > ("CollisionModel")) {
            col_model = "Enabled";
        }
        else {
            col_model = "Disabled";
        }
        QString ref_frame_str = "";
        if (!data.at< QString > ("Ref. frame").isEmpty ()) {
            ref_frame_str = " refframe=\"" + data.at< QString > ("Ref. frame") + "\"";
        }
        else {
            ref_frame_str = "";
        }

        // If the drawable is defined inside a frame, we indent with an
        // additional tab
        QString indentation = "";
        if (ref_frame_str.isEmpty ())
            indentation = "    ";
        else
            indentation = "  ";

        QString model_str = "";

        if (data.at< QString > ("CAD file").isEmpty ()) {
            // Geometry I presume?
            if (data.at< QString > ("Geometry Type") == "Box") {
                QTextStream (&model_str)
                    << indentation << "  <Box x=\"" << data.at< QVector3D > ("Box (x, y, z)").x ()
                    << "\" y=\"" << data.at< QVector3D > ("Box (x, y, z)").y () << "\" z=\""
                    << data.at< QVector3D > ("Box (x, y, z)").z () << "\"/>";
            }
            else if (data.at< QString > ("Geometry Type") == "Cone") {
                QTextStream (&model_str)
                    << indentation << "  <Cone radius=\""
                    << data.at< QVector2D > ("Cone (radiusBot, height)").x () << "\" z=\""
                    << data.at< QVector2D > ("Cone (radiusBot, height)").y () << "\" level=\""
                    << data.at< int > ("Mesh Resolution") << "\"/>";
            }
            else if (data.at< QString > ("Geometry Type") == "Cylinder") {
                QTextStream (&model_str)
                    << indentation << "  <Cylinder radius=\""
                    << data.at< QVector2D > ("Cylinder (radius, height)").x () << "\" z=\""
                    << data.at< QVector2D > ("Cylinder (radius, height)").y () << "\" level=\""
                    << data.at< int > ("Mesh Resolution") << "\"/>";
            }
            else if (data.at< QString > ("Geometry Type") == "Sphere") {
                QTextStream (&model_str)
                    << indentation << "  <Sphere radius=\"" << data.at< double > ("Sphere (r)")
                    << "\" level=\"" << data.at< int > ("Mesh Resolution") << "\"/>";
            }
        }
        else {
            QTextStream (&model_str) << indentation << "  <Polytope file=\""
                                     << data.at< QString > ("CAD file") << "\"/>";
        }

        QString drawable_str;
        QTextStream (&drawable_str)
            << "<Drawable name=\"" << data.at< QString > ("Name") << "\"" << ref_frame_str
            << " colmodel=\"" << col_model << "\">"
            << "\n"
            <<
            // POS
            indentation << "  <Pos>" << data.at< QVector3D > ("Position").x () << " "
            << data.at< QVector3D > ("Position").y () << " "
            << data.at< QVector3D > ("Position").z ()
            << "</Pos>\n"
            // RPY
            << indentation << "  <RPY>" << data.at< QVector3D > ("RPY").x () << " "
            << data.at< QVector3D > ("RPY").y () << " " << data.at< QVector3D > ("RPY").z ()
            << "</RPY>\n"
            // RGB
            << indentation << "  <RGB>" << data.at< QVector3D > ("Color (r,g,b)").x () << " "
            << data.at< QVector3D > ("Color (r,g,b)").y () << " "
            << data.at< QVector3D > ("Color (r,g,b)").z ()
            << " </RGB>\n"
            // MODEL
            << model_str << "\n"
            << indentation << "</Drawable>";

        // Insert frame at cursor position
        getCurrentTab ()->_editor->insertXMLTextUnderCursor (drawable_str);
    }
}

void WorkcellEditorWindow::on_actionAdd_CollisionModel_triggered (bool)
{
    std::string defaultName = _pmap.get< std::string > ("WorkcellFile", "");
    std::size_t found       = defaultName.find_last_of ("/\\");
    std::string cad_path    = defaultName.substr (0, found);

    // Define form options
    InputFormDialog::FormOptions options;
    options.listDisplaysAsRadios = false;
    options.listReturnsIndex     = false;

    // Define form inputs
    InputFormDialog::FormData data;
    data["Name"]          = "";
    QStringList refFrames = getRefFrameList ();
    if (refFrames.isEmpty ()) {
        std::cout << "wc is null" << std::endl;
        data["Ref. frame"] = "";
    }
    else {
        data["Ref. frame"] = refFrames;
    }
    data["Mesh Resolution"] = 20;
    data["Position"]        = QVector3D (0.0, 0.0, 0.0);
    data["RPY"]             = QVector3D (0.0, 0.0, 0.0);
    data["CAD file"]        = "cad_file," + QString::fromStdString (cad_path);
    data["Geometry Type"]   = QStringList () << "Box"
                                           << "Cone"
                                           << "Cylinder"
                                           << "Sphere";
    data["Box (x, y, z)"]             = QVector3D (0.0, 0.0, 0.0);
    data["Cone (radiusBot, height)"]  = QVector2D (0.0, 0.0);
    data["Cylinder (radius, height)"] = QVector2D (0.0, 0.0);
    data["Sphere (r)"]                = 0.0;

    // Ask user for input and retrieve data
    if (InputFormDialog::getInput ("Add Collision Model", data, options)) {
        QString ref_frame_str = "";
        if (!data.at< QString > ("Ref. frame").isEmpty ())
            ref_frame_str = " refframe=\"" + data.at< QString > ("Ref. frame") + "\"";
        else
            ref_frame_str = "";

        // If the drawable is defined inside a frame, we indent with an
        // additional tab
        QString indentation = "";
        if (ref_frame_str.isEmpty ())
            indentation = "    ";
        else
            indentation = "  ";

        QString model_str = "";
        if (data.at< QString > ("CAD file").isEmpty ()) {
            // Geometry I presume?
            if (data.at< QString > ("Geometry Type") == "Box") {
                QTextStream (&model_str)
                    << "<Box x=\"" << data.at< QVector3D > ("Box (x, y, z)").x () << "\" y=\""
                    << data.at< QVector3D > ("Box (x, y, z)").y () << "\" z=\""
                    << data.at< QVector3D > ("Box (x, y, z)").z () << "\"/>";
            }
            else if (data.at< QString > ("Geometry Type") == "Cone") {
                QTextStream (&model_str)
                    << "<Cone radius=\"" << data.at< QVector2D > ("Cone (radiusBot, height)").x ()
                    << "\" z=\"" << data.at< QVector2D > ("Cone (radiusBot, height)").y ()
                    << "\" level=\"" << data.at< int > ("Mesh Resolution") << "\"/>";
            }
            else if (data.at< QString > ("Geometry Type") == "Cylinder") {
                QTextStream (&model_str)
                    << "<Cylinder radius=\""
                    << data.at< QVector2D > ("Cylinder (radius, height)").x () << "\" z=\""
                    << data.at< QVector2D > ("Cylinder (radius, height)").y () << "\" level=\""
                    << data.at< int > ("Mesh Resolution") << "\"/>";
            }
            else if (data.at< QString > ("Geometry Type") == "Sphere") {
                QTextStream (&model_str)
                    << "<Sphere radius=\"" << data.at< double > ("Sphere (r)") << "\" level=\""
                    << data.at< int > ("Mesh Resolution") << "\"/>";
            }
        }
        else {
            QTextStream (&model_str)
                << "<Polytope file=\"" << data.at< QString > ("CAD file") << "\"/>";
        }

        QString col_model_str;
        QTextStream (&col_model_str)
            << "<CollisionModel name=\"" << data.at< QString > ("Name") << "\"" << ref_frame_str
            << ">"
            << "\n"
            // POS
            << indentation << "  <Pos>" << data.at< QVector3D > ("Position").x () << " "
            << data.at< QVector3D > ("Position").y () << " "
            << data.at< QVector3D > ("Position").z ()
            << "</Pos>\n"
            // RPY
            << indentation << "  <RPY>" << data.at< QVector3D > ("RPY").x () << " "
            << data.at< QVector3D > ("RPY").y () << " " << data.at< QVector3D > ("RPY").z ()
            << "</RPY>"
            << "\n"
            // Model
            << indentation << "  " << model_str << "\n"
            << indentation << "</CollisionModel>";

        // Insert frame at cursor position
        getCurrentTab ()->_editor->insertXMLTextUnderCursor (col_model_str);
    }
}

bool WorkcellEditorWindow::save ()
{
    // save the current viable
    std::string filename = getCurrentTab ()->_filename;
    if (filename == "") {
        return saveAs ();
    }
    else {
        return save (filename);
    }
}

bool WorkcellEditorWindow::saveAs ()
{
    std::string defaultName = _pmap.get< std::string > ("WorkcellFile", "");
    QString filename        = QFileDialog::getSaveFileName (
        this, "Save as", defaultName.c_str (), "Supported (*.wc.xml *.xml)");

    if (filename.isEmpty ())
        return false;

    return save (filename.toStdString ());
}

bool WorkcellEditorWindow::save (const std::string& filename)
{
    bool isTmpFile = false;
    if (filename.find (".temp.wc.xml") < filename.size ()) {
        isTmpFile = true;
    }
    QFile file;
    file.setFileName (filename.c_str ());
    if (file.open (QFile::WriteOnly | QFile::Text)) {
        {
            QTextStream out (&file);
            out << getCurrentTab ()->_editor->toPlainText ();
            if (!isTmpFile) {
                _pmap.set< std::string > ("WorkcellFile", filename);
                getCurrentTab ()->_editor->document ()->setModified (false);
                getCurrentTab ()->_filename = filename;

                _tabPane->setTabText (
                    _tabPane->indexOf (getCurrentTab ()->_editor),
                    boost::filesystem::path (filename).filename ().string ().c_str ());
            }
        }
        file.close ();

        // Verify workcell xml using schemas
        /*try {
            XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
        } catch( XMLException& e ){
            RW_THROW("Xerces initialization
        Error"<<XMLStr(e.getMessage()).str());
        }

        XercesDOMParser parser;
        xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser,
        filename,
         "/home/prier/RobWork/RobWork/xml-schemas/rwxml_workcell.xsd");
        */

        rw::core::Log ().setLevel (rw::core::Log::Debug);
        rw::models::WorkCell::Ptr wc = nullptr;

        if (!isTmpFile) {
            std::string wcFilename = _pmap.get< std::string > ("WorkcellFile", "");
            wc                     = rw::loaders::WorkCellLoader::Factory::load (wcFilename);
        }
        else {
            wc = rw::loaders::WorkCellLoader::Factory::load (filename);
            file.remove ();
        }
        rw::core::Log ().setLevel (rw::core::Log::Info);

        if (_rws != nullptr) {
            ignoreNextWorkcellOpen = true;
            _rws->setWorkcell (wc);
        }

        return true;
    }
    else {
        file.close ();
        return false;
    }
}

bool WorkcellEditorWindow::openWorkCell (const QString& fileName)
{
    bool success = false;
    if (!fileName.isEmpty () && !ignoreNextWorkcellOpen) {
        _pmap.set< std::string > ("PreviousOpenDirectory",
                                  StringUtil::getDirectoryName (fileName.toStdString ()));
        _pmap.set< std::string > ("WorkcellFile", fileName.toStdString ());
        QFile file;
        file.setFileName (fileName);
        EditorTab::Ptr tab = makeEditor ();
        tab->_filename     = file.fileName ().toStdString ();
        _tabPane->setTabText (
            _tabPane->indexOf (tab->_editor),
            boost::filesystem::path (fileName.toStdString ()).filename ().string ().c_str ());

        if (file.open (QFile::ReadWrite | QFile::Text)) {
            tab->_editor->setPlainText (file.readAll ());
            success = true;
        }
        file.close ();
        tab->_editor->document ()->setModified (false);
        _tabPane->setCurrentIndex (_tabPane->indexOf (tab->_editor));
    }
    else if (ignoreNextWorkcellOpen) {
        ignoreNextWorkcellOpen = false;
    }

    return success;
}

QStringList WorkcellEditorWindow::getRefFrameList ()
{
    EditorTab::Ptr tab           = getCurrentTab ();
    rw::models::WorkCell::Ptr wc = NULL;
    QStringList refFrames;
    refFrames << "";
    if (tab->_filename.size () > 5 /*minimum length*/) {
        std::string file = tab->_filename + ".temp1234.xml";

        std::ofstream out (file);
        out << tab->_editor->toPlainText ().toStdString ();
        out.close ();

        wc = WorkCellLoader::Factory::load (file);
        remove (file.c_str ());

        if (!wc.isNull ()) {
            std::vector< rw::kinematics::Frame* > frameList = wc->getFrames ();
            for (size_t i = 0; i < frameList.size (); i++) {
                refFrames << frameList[i]->getName ().c_str ();
            }
        }
    }
    return refFrames;
}

WorkcellEditorWindow::EditorTab::Ptr WorkcellEditorWindow::getCurrentTab ()
{
    return _editors[_tabPane->currentWidget ()];
}

void WorkcellEditorWindow::on_actionRun_triggered (bool)
{
    // Save as temproary file
    save (getCurrentTab ()->_filename + ".temp.wc.xml");
}

void WorkcellEditorWindow::on_actionReload_triggered (bool)
{
    std::string fileName = getCurrentTab ()->_filename;
    if (fileName != "") {
        QFile file;
        file.setFileName (fileName.c_str ());
        if (file.open (QFile::ReadWrite | QFile::Text)) {
            getCurrentTab ()->_editor->setPlainText (file.readAll ());
        }
        file.close ();
    }
}

QAbstractItemModel* WorkcellEditorWindow::modelFromFile (const QString& fileName,
                                                         WCECompleter* completer)
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

        const QRegularExpression re ("^\\s+");
        const QRegularExpressionMatch match = re.match(line);
        int level = 0;
        if (!match.hasMatch()) {
            level = 0;
        }
        else {
            if (line.startsWith ("\t")) {
                level = match.capturedLength(0);
            }
            else {
                level = match.capturedLength(0) / 4;
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
