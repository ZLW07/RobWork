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

#ifndef PROPERTYVIEWEDITOR_HPP_
#define PROPERTYVIEWEDITOR_HPP_

#include <rw/core/Ptr.hpp>

#include <QObject>
#include <QtTreePropertyBrowser>

namespace rw { namespace core {
    class PropertyBase;
}}    // namespace rw::core
namespace rw { namespace core {
    class PropertyMap;
}}    // namespace rw::core

class QtProperty;
class QtVariantEditorFactory;
class QtVariantPropertyManager;

class QWidget;

//! @brief Qt widget for editing a PropertyMap.
class PropertyViewEditor : public QtTreePropertyBrowser
{
    Q_OBJECT
  public:
    /**
     * @brief Make new editor.
     * @param parent [in] the owning parent widget.
     */
    PropertyViewEditor (QWidget* parent);

    //! @brief Destructor.
    virtual ~PropertyViewEditor ();

    /**
     * @brief set the propertymap and update it.
     * @param map
     */
    void setPropertyMap (rw::core::Ptr< rw::core::PropertyMap > map)
    {
        _map = map;
        update ();
    }

    /**
     * @brief updates the propertyviewer with its propertymap
     */
    void update ();

    /**
     * @brief sets the number of decimal places used for displaying properties of
     *	 floats and double
     * @param decimals
     *
     * Note that the update() must be called in order for the changes to take effect
     */
    void setDecimals (const int decimals) { _decimals = decimals; };

  Q_SIGNALS:
    /**
     * @brief if a property is changed then its identifier is signalled.
     */
    void propertyChanged (const std::string& identifier);

  private Q_SLOTS:
    void slotValueChanged (QtProperty* property, const QVariant& value);
    void slotPropertyDestroyed (QtProperty* property);

  private:
    QtProperty* update (rw::core::Ptr< rw::core::PropertyMap > map, std::string propname);

  private:
    QtVariantEditorFactory* _variantFactory;
    QtVariantPropertyManager* _variantManager;
    rw::core::Ptr< rw::core::PropertyMap > _map;
    std::map< QtProperty*, rw::core::Ptr< rw::core::PropertyBase > > _qtPropToRwProp;
    std::map< QtProperty*, rw::core::Ptr< rw::core::PropertyMap > > _qtPropToRwPropMap;

    int _decimals;
};

#endif /* PROPERTYVIEWDIALOG_HPP_ */
