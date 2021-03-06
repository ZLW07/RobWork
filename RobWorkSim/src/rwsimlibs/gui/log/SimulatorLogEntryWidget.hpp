/******************************************************************************
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
 ******************************************************************************/

#ifndef RWSIMLIBS_GUI_SIMULATORLOGENTRYWIDGET_HPP_
#define RWSIMLIBS_GUI_SIMULATORLOGENTRYWIDGET_HPP_

/**
 * @file SimulatorLogEntryWidget.hpp
 *
 * \copydoc rwsimlibs::gui::SimulatorLogEntryWidget
 */

#include <rw/core/ExtensionPoint.hpp>

#include <QWidget>

namespace rw { namespace core {
    class PropertyMap;
}}    // namespace rw::core
namespace rw { namespace graphics {
    class GroupNode;
}}    // namespace rw::graphics
namespace rw { namespace graphics {
    class SceneGraph;
}}    // namespace rw::graphics
namespace rwsim { namespace dynamics {
    class DynamicWorkCell;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace log {
    class SimulatorLog;
}}    // namespace rwsim::log

namespace rwsimlibs { namespace gui {
    //! @addtogroup rwsimlibs_gui

    //! @{
    //! @brief A widget that is a graphical representation of a rwsim::log::SimulatorLogEntry.
    class SimulatorLogEntryWidget : public QWidget
    {
        Q_OBJECT
      public:
        /**
         * @brief Construct new widget for a log entry.
         * @param parent [in] (optional) the parent Qt widget. Ownership is shared by the caller and
         * the parent widget if given.
         */
        SimulatorLogEntryWidget (QWidget* parent = 0);

        //! @brief Destructor.
        virtual ~SimulatorLogEntryWidget ();

        /**
         * @brief Set the dynamic workcell for visualisation of the log information.
         * @param dwc [in] the dynamic workcell.
         */
        virtual void setDWC (rw::core::Ptr< const rwsim::dynamics::DynamicWorkCell > dwc) = 0;

        /**
         * @brief Set the log entity to show.
         * @param entry [in] the log entity.
         */
        virtual void setEntry (rw::core::Ptr< const rwsim::log::SimulatorLog > entry) = 0;

        /**
         * @brief Get the current log entity.
         * @return the log entity.
         */
        virtual rw::core::Ptr< const rwsim::log::SimulatorLog > getEntry () const = 0;

        /**
         * @brief Re-read logging data and update widget.
         */
        virtual void updateEntryWidget () = 0;

        /**
         * @brief Add graphics as drawables to a scene-graph.
         * @param root [in] the node to add drawables to.
         * @param graph [in] the scene graph.
         */
        virtual void showGraphics (rw::core::Ptr< rw::graphics::GroupNode > root,
                                   rw::core::Ptr< rw::graphics::SceneGraph > graph) = 0;

        /**
         * @brief Get the name of the widget (will be used as the name on tabs).
         * @return the name.
         */
        virtual std::string getName () const = 0;

        /**
         * @brief Set properties for widget.
         * @param properties [in/out] properties, such as default values for scaling of graphical
         * elements.
         */
        virtual void setProperties (rw::core::Ptr< rw::core::PropertyMap > properties);

        //! @brief Dispatchers are responsible for creating new widgets of type
        //! SimulatorLogEntryWidget.
        class Dispatcher
        {
          public:
            //! @brief Smart pointer to a dispatcher.
            typedef rw::core::Ptr< const Dispatcher > Ptr;

            //! @brief Destructor.
            virtual ~Dispatcher (){}

            /**
             * @brief Make new SimulatorLogEntryWidget
             * @param entry [in] the entry to make widget for.
             * @param parent [in] the parent Qt widget (optional).
             * @return a pointer to the widget - ownership is shared by the caller and the parent
             * widget if given.
             */
            virtual SimulatorLogEntryWidget*
            makeWidget (rw::core::Ptr< const rwsim::log::SimulatorLog > entry,
                        QWidget* parent = 0) const = 0;

            /**
             * @brief Check if the widget created by this dispatcher will be work for the given log
             * entry.
             * @param entry [in] the entry.
             * @return true if widgets will work for this entry, or false otherwise.
             */
            virtual bool accepts (rw::core::Ptr< const rwsim::log::SimulatorLog > entry) const = 0;

          protected:
            Dispatcher (){};
        };

        /**
         * @addtogroup extensionpoints
         * @extensionpoint{rwsimlibs::gui::SimulatorLogEntryWidget::Factory,rwsimlibs::gui::SimulatorLogEntryWidget::Dispatcher,rwsimlibs.gui.SimulatorLogEntryWidget}
         */

        /**
         * @brief A factory for a SimulatorLogEntryWidget. This factory also defines an
         * extension point for SimulatorLogEntryWidget::Dispatcher.
         */
        class Factory : public rw::core::ExtensionPoint< SimulatorLogEntryWidget::Dispatcher >
        {
          public:
            /**
             * @brief Get a list of widget dispatchers that accepts the given entry.
             * @param entry [in] the entry to match.
             * @return the list of dispatchers.
             */
            static std::list< SimulatorLogEntryWidget::Dispatcher::Ptr >
            getWidgetDispatchers (rw::core::Ptr< const rwsim::log::SimulatorLog > entry);

            /**
             * @brief Create widget(s) for the given entry.
             * @param entry [in] the entry to match.
             * @param parent [in] the parent Qt widget (optional).
             * @return a list of new widgets - ownership is shared by the caller and the parent
             * widget if given.
             */
            static std::list< SimulatorLogEntryWidget* >
            makeWidgets (rw::core::Ptr< const rwsim::log::SimulatorLog > entry,
                         QWidget* parent = 0);

          private:
            Factory ();
        };

      signals:
        //! @brief Signal is emitted if the graphics is updated.
        void graphicsUpdated ();

      protected:
        //! @brief Widget properties.
        rw::core::Ptr< rw::core::PropertyMap > _properties;
    };
    //! @}
}}     // namespace rwsimlibs::gui
#endif /* RWSIMLIBS_GUI_SIMULATORLOGENTRYWIDGET_HPP_ */
