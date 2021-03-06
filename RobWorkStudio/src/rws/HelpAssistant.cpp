
// this stuff is very much borrowed from the Qt example homepage

#include "HelpAssistant.hpp"

#include "../RobWorkStudioConfig.hpp"

#include <QMessageBox>
#include <QtCore/QByteArray>
#include <QtCore/QDir>
#include <QtCore/QLibraryInfo>
#include <QtCore/QProcess>

HelpAssistant::HelpAssistant () : proc (0)
{}

HelpAssistant::~HelpAssistant ()
{
    if (proc && proc->state () == QProcess::Running) {
        proc->terminate ();
        proc->waitForFinished (3000);
    }
    delete proc;
}

bool HelpAssistant::showDocumentation (const QStringList& paths)
{
    QStringList files;
    // search for the help file
    QString filename ("robwork_help-v");
    filename.append (RWS_VERSION);
    filename.append (".qhc");
    files.append (filename);
    files.append ("docs/" + filename);
    files.append ("../docs/" + filename);
    files.append ("../../docs/" + filename);

    QString absfilename;
    for (int j = 0; j < paths.size (); j++) {
        QString path = paths[j];

        for (int i = 0; i < files.size (); i++) {
            QString file = path;
            file.append ("/");
            file.append (files[i]);
            if (QFile::exists (file)) {
                absfilename = file;
                break;
            }
            // std::cout << std::endl;
        }
        if (absfilename.size () != 0)
            break;
    }

    if (absfilename.size () == 0) {
        QMessageBox msgBox;
        msgBox.setText ("The RobWork help files could not be located. \nMake sure they are "
                        "properly installed.");
        msgBox.exec ();
        return false;
    }

    return startAssistant (absfilename);
}

void HelpAssistant::gotoURL (const std::string& url)
{
    if (proc->state () != QProcess::Running)
        return;
    QByteArray ba ("SetSource ");
    ba.append (url.c_str ());
    ba.append ('\n');
    proc->write (ba);
}

void HelpAssistant::minimumView ()
{
    if (proc->state () != QProcess::Running)
        return;
    QByteArray ba;
    ba.append ("hide search;");
    ba.append ("hide bookmarks;");
    ba.append ("hide index;");
    ba.append ("hide contents");
    ba.append ('\n');
    proc->write (ba);
}

bool HelpAssistant::startAssistant (QString collectionFileName)
{
    if (!proc)
        proc = new QProcess ();

    if (proc->state () != QProcess::Running) {
#if QT_VERSION >= 0x060000
        QString app = QLibraryInfo::path (QLibraryInfo::BinariesPath) + QDir::separator ();
#else
        QString app = QLibraryInfo::location (QLibraryInfo::BinariesPath) + QDir::separator ();
#endif
#if !defined(Q_OS_MAC)
        app += QLatin1String ("assistant");
#else
        app += QLatin1String ("Assistant.app/Contents/MacOS/Assistant");
#endif

        QStringList args;
        args << QLatin1String ("-collectionFile")
             << QLatin1String (collectionFileName.toStdString ().c_str ())
             << QLatin1String ("-enableRemoteControl");

        proc->start (app, args);

        if (!proc->waitForStarted ()) {
            QMessageBox::critical (0,
                                   QObject::tr ("RobWork Help"),
                                   QObject::tr ("Unable to launch Qt Assistant (%1)").arg (app));
            return false;
        }
    }
    return true;
}
