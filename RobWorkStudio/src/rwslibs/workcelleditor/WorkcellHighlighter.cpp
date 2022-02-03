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

#include "WorkcellHighlighter.hpp"

#include <QRegularExpression>
#include <QTextCharFormat>

WorkcellHighlighter::HighlightingRule WorkcellHighlighter::makeRule (QString pattern, QColor color,
                                                                     int fontweight)
{
    HighlightingRule rule;
    QTextCharFormat format;
    format.setForeground (color);
    format.setFontWeight (fontweight);
    rule.pattern = QRegularExpression (pattern);
    rule.format  = format;
    return rule;
}

WorkcellHighlighter::WorkcellHighlighter (QTextDocument* parent) : QSyntaxHighlighter (parent)
{
    HighlightingRule rule;

    highlightingRules.append (makeRule ("\\bWorkCell\\b", QColor (0, 94, 136), QFont::Bold));

    highlightingRules.append (makeRule ("\\bFrame\\b", QColor (217, 0, 93), QFont::Bold));

    highlightingRules.append (makeRule ("\\bDrawable\\b", QColor (0, 175, 95), QFont::Bold));

    highlightingRules.append (makeRule ("\\bCollisionModel\\b", QColor (255, 87, 79), QFont::Bold));

    highlightingRules.append (makeRule ("\\bProperty\\b", Qt::darkGreen, QFont::Bold));

    highlightingRules.append (makeRule ("\\bSerialDevice\\b", QColor (1, 135, 134), QFont::Bold));

    highlightingRules.append (makeRule ("\\bInclude\\b", QColor (46, 125, 50), QFont::Bold));

    highlightingRules.append (makeRule ("\\bRPY\\b", QColor (142, 93, 171), QFont::Bold));
    highlightingRules.append (makeRule ("\\bPos\\b", QColor (142, 93, 171), QFont::Bold));
    highlightingRules.append (makeRule ("\\bPolytope\\b", QColor (142, 93, 171), QFont::Bold));
    highlightingRules.append (makeRule ("\\bBox\\b", QColor (142, 93, 171), QFont::Bold));
    highlightingRules.append (makeRule ("\\bRGB\\b", QColor (142, 93, 171), QFont::Bold));

    classFormat.setFontWeight (QFont::Bold);
    classFormat.setForeground (Qt::darkMagenta);
    rule.pattern = QRegularExpression ("\\bQ[A-Za-z]+\\b");
    rule.format  = classFormat;
    highlightingRules.append (rule);

    singleLineCommentFormat.setForeground (QColor (135, 135, 135));
    rule.pattern = QRegularExpression ("--(?!\\[)[^\n]*");
    rule.format  = singleLineCommentFormat;
    highlightingRules.append (rule);

    multiLineCommentFormat.setForeground (Qt::red);
    quotationFormat.setForeground (Qt::darkGreen);

    attributeFormat.setForeground (QColor (215, 95, 0));
    rule.pattern = QRegularExpression (R"**((?<range2>[\w\d\-\:]+)[ ]*=[ ]*"[^"]*")**",
                                       QRegularExpression::DotMatchesEverythingOption |
                                           QRegularExpression::MultilineOption);
    rule.format  = attributeFormat;
    highlightingRules.append (rule);

    rule.pattern = QRegularExpression (R"**((?<!\\)([\"'])(.+?)(?<!\\)\1)**",
                                       QRegularExpression::DotMatchesEverythingOption |
                                           QRegularExpression::MultilineOption);
    rule.format  = quotationFormat;
    highlightingRules.append (rule);

    functionFormat.setFontItalic (true);
    functionFormat.setForeground (Qt::blue);
    rule.pattern = QRegularExpression ("\\b[A-Za-z0-9_]+(?=\\()");
    rule.format  = functionFormat;
    highlightingRules.append (rule);

    commentStartExpression = QRegularExpression ("--\\[");
    commentEndExpression   = QRegularExpression ("\\]");
}

void WorkcellHighlighter::highlightBlock (const QString& text)
{
    Q_FOREACH (const HighlightingRule& rule, highlightingRules) {
        QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch (text);
        while (matchIterator.hasNext ()) {
            QRegularExpressionMatch match = matchIterator.next ();
            setFormat (match.capturedStart (), match.capturedLength (), rule.format);
        }
    }
    setCurrentBlockState (0);

    int startIndex = 0;
    if (previousBlockState () != 1)
        startIndex = text.indexOf (commentStartExpression);

    while (startIndex >= 0) {
        QRegularExpressionMatch match = commentEndExpression.match (text, startIndex);
        int endIndex                  = match.capturedStart ();
        int commentLength             = 0;
        if (endIndex == -1) {
            setCurrentBlockState (1);
            commentLength = text.length () - startIndex;
        }
        else {
            commentLength = endIndex - startIndex + match.capturedLength ();
        }
        setFormat (startIndex, commentLength, multiLineCommentFormat);
        startIndex = text.indexOf (commentStartExpression, startIndex + commentLength);
    }
}
