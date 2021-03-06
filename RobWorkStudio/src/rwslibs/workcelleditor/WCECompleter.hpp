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

#ifndef WCECOMPLETER_HPP_
#define WCECOMPLETER_HPP_

#include <QCompleter>

class WCECompleter : public QCompleter
{
    Q_OBJECT
    Q_PROPERTY (QString separator READ separator WRITE setSeparator)

  public:
    WCECompleter (QObject* parent = 0);

    WCECompleter (QAbstractItemModel* model, QObject* parent = 0);

    WCECompleter (QStringList list, QObject* paraent = 0);

    QString separator () const;

  public Q_SLOTS:

    void setSeparator (const QString& separator);

  protected:
    QStringList splitPath (const QString& path) const;

    QString pathFromIndex (const QModelIndex& index) const;

  private:
    QString sep;
};

#endif /* WCECompleter_HPP_ */
