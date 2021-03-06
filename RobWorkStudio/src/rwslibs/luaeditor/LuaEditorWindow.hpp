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

#ifndef LUAEDITORWINDOW_HPP_
#define LUAEDITORWINDOW_HPP_

#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>

#include <QMainWindow>
#include <map>

namespace rw { namespace core {
    class Log;
}}    // namespace rw::core
namespace rwlibs { namespace swig {
    class LuaState;
}}    // namespace rwlibs::swig

class CodeEditor;
class LuaHighlighter;

namespace Ui {
class LuaEditorWindow;
}

class TreeModelCompleter;
class QAbstractItemModel;

namespace rws {
class RobWorkStudio;
class LuaExecutionThread;

/**
 * @brief A lua editor and programming pad designed to resemble a teach pendent
 * for devices in RobWorkStudio.
 *
 * This editor will enable lua scripting and execution from within RobWorkStudio.
 */
class LuaEditorWindow : public QMainWindow
{
    Q_OBJECT
  public:
    /**
     * @brief Constructor
     * @param lua [in] the lua state on which to operate
     * @param output [in] the log on which to stream print functionality and errors
     * @param rwstudio [in] instance of RobWorkStudio
     * @param parent [in] the Qt parent widget
     */
    LuaEditorWindow (rw::core::Ptr< rwlibs::swig::LuaState > lua,
                     rw::core::Ptr< rw::core::Log > output, rws::RobWorkStudio* rwstudio,
                     QWidget* parent);

    //! @brief destructor
    virtual ~LuaEditorWindow ();

    /**
     * @brief change the lua state
     * @param lua [in] the new lua state which is to be used.
     */
    void setLuaState (rw::core::Ptr< rwlibs::swig::LuaState > lua) { _lua = lua; }

  public Q_SLOTS:
    void on_actionNew_triggered (bool);
    void on_actionOpen_triggered (bool);
    void on_actionSave_triggered (bool);
    void on_actionSave_As_triggered (bool);
    void on_actionRun_triggered (bool);
    void on_actionStop_triggered (bool);
    void on_actionReset_triggered (bool);
    void on_actionReload_triggered (bool);
    void on_actionClose_triggered (bool);

    void textChanged ();
    void runFinished ();
    void ShowContextMenu (const QPoint& p);
    void setCheckAction (QAction*);

  private:
    QAbstractItemModel* modelFromFile (const QString& fileName, TreeModelCompleter* completer);

    struct EditorTab
    {
        typedef rw::core::Ptr< EditorTab > Ptr;
        std::string _id;
        CodeEditor* _editor;
        LuaHighlighter* _highlighter;
        TreeModelCompleter* _completer;
        std::string _filename;
    };

    EditorTab::Ptr makeEditor ();

    bool save ();
    bool saveAs ();
    bool save (const std::string& filename);
    EditorTab::Ptr getCurrentTab ();

  private:
    //! hold
    std::map< QWidget*, EditorTab::Ptr > _editors;
    class Ui::LuaEditorWindow* _ui;

    rw::core::Ptr< rwlibs::swig::LuaState > _lua;
    rw::core::Ptr< rw::core::Log > _output;
    rw::core::PropertyMap _pmap;

    bool _isRunning;

    QTabWidget* _tabPane;
    LuaExecutionThread* _luaRunner;
    rws::RobWorkStudio* _rws;
};

}    // namespace rws

#endif /* LUAEDITORWINDOW_HPP_ */
