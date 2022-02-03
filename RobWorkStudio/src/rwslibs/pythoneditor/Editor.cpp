#include "Editor.hpp"

#include <RobWorkStudio.hpp>

#include <QCodeEditor>
#include <QLayout>
#include <QPushButton>
#include <QPythonCompleter>
#include <QPythonHighlighter>
#include <QSyntaxStyle>
#include <QWidget>
#include <boost/bind.hpp>

using rw::kinematics::State;
using rw::models::WorkCell;
using rws::PyEditor;
using rws::RobWorkStudioPlugin;

PyEditor::PyEditor () : RobWorkStudioPlugin ("PyEditor", QIcon (":/icon.png"))
{
    auto container = new QWidget (this);
    this->setWidget (container);

    auto main_layout = new QVBoxLayout (container);
    main_layout->setSpacing (0);
    main_layout->setContentsMargins (5, 0, 0, 0);

    // Menu btns
    auto menu        = new QWidget (container);
    auto menu_layout = new QHBoxLayout (menu);
    menu_layout->setSpacing (0);
    menu->setLayout (menu_layout);
    menu_layout->setContentsMargins (0, 0, 0, 0);

    // Run btn
    QPixmap go_pm (":/forward.png");
    QIcon go_ico (go_pm);
    QPushButton* play_btn = new QPushButton (menu);
    play_btn->setIcon (go_ico);
    play_btn->setIconSize (QSize (24, 24));
    menu_layout->addWidget (play_btn);
    menu_layout->addStretch ();

    main_layout->addWidget (menu);
    // CodeEditor
    _editor = new QCodeEditor (container);
    main_layout->addWidget (_editor);

    _editor->setPlainText ("from sdurw import *\nfrom sdurws import *\n");
    _editor->setSyntaxStyle (QSyntaxStyle::defaultStyle ());
    _editor->setCompleter (new QPythonCompleter (this));
    _editor->setHighlighter (new QPythonHighlighter);

    // Connecting components
    connect (play_btn, SIGNAL (clicked ()), this, SLOT (runCode ()));
}

PyEditor::~PyEditor ()
{
    delete _editor;
}

void PyEditor::initialize ()
{
    getRobWorkStudio ()->stateChangedEvent ().add (
        boost::bind (&PyEditor::stateChangedListener, this, boost::arg< 1 > ()), this);
}

void PyEditor::open (WorkCell* workcell)
{}

void PyEditor::close ()
{}

void PyEditor::stateChangedListener (const State& state)
{}

void PyEditor::runCode ()
{
    _pyrun.runCode (_editor->toPlainText ().toStdString ());
}