#include "RWSSpinBox.hpp"

#include <QLocale>
#include <QString>

using namespace rws;
RWSSpinBox::RWSSpinBox (double low, double high)
{
    this->setDecimals (3);
    this->setRange (low, high);
    this->setKeyboardTracking (true);
    const double step = (high - low) / 400;
    this->setSingleStep (step);
}

void RWSSpinBox::fixup (QString& input) const
{
    QString out;
    int deci    = decimals ();
    bool gotNum = false;
    bool dot    = false;
    for (int i = 0; i < input.count (); i++) {
        if (deci == 0) {
            break;
        }
        if (input[i] == '-' && !gotNum) {
            out += input[i];
            gotNum = true;
        }
        else if (input[i] >= '0' && input[i] <= '9') {
            if (dot) {
                deci--;
            }
            out += input[i];
            gotNum = true;
        }
        else if ((input[i] == '.' || input[i] == ',') && !dot) {
            out += locale().decimalPoint();
            dot = true;
        }
    }
    input = out;
}

QValidator::State RWSSpinBox::validate (QString& text, int& pos) const
{
    
    bool ok = false;
    QString t = text;
    fixup (t);
    locale ().toDouble (t, &ok);
    return ok ? QValidator::Acceptable : QValidator::Invalid;
}

double RWSSpinBox::valueFromText (const QString& text) const
{
    bool ok   = false;
    QString t = text;
    fixup (t);
    
    double value = locale ().toDouble (t, &ok);
    return ok ? value : QDoubleSpinBox::value ();
}