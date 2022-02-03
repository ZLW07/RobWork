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

#include "LuaHighlighter.hpp"

LuaHighlighter::LuaHighlighter (QTextDocument* parent) : QSyntaxHighlighter (parent)
{
    HighlightingRule rule;

    keywordFormat.setForeground (Qt::darkBlue);
    keywordFormat.setFontWeight (QFont::Bold);
    QStringList keywordPatterns;

    keywordPatterns << "\\bdo\\b"
                    << "\\bend\\b"
                    << "\\bwhile\\b"
                    << "\\brepeat\\b"
                    << "\\buntil\\b"
                    << "\\bif\\b"
                    << "\\belseif\\b"
                    << "\\belse\\b"
                    << "\\bfor\\b"
                    << "\\bin\\b"
                    << "\\bfunction\\b"
                    << "\\blocal\\b"
                    << "\\bnil\\b"
                    << "\\bfalse\\b"
                    << "\\btrue\\b"
                    << "\\band\\b"
                    << "\\bor\\b"
                    << "\\bnot\\b"
                    << "\\bthen\\b"
                    << "\\bbreak\\b";

    Q_FOREACH (const QString& pattern, keywordPatterns) {
        rule.pattern = QRegularExpression (pattern);
        rule.format  = keywordFormat;
        highlightingRules.append (rule);
    }

    classFormat.setFontWeight (QFont::Bold);
    classFormat.setForeground (Qt::darkMagenta);
    rule.pattern = QRegularExpression ("\\bQ[A-Za-z]+\\b");
    rule.format  = classFormat;
    highlightingRules.append (rule);

    singleLineCommentFormat.setForeground (Qt::red);
    rule.pattern = QRegularExpression ("--(?!\\[)[^\n]*");
    rule.format  = singleLineCommentFormat;
    highlightingRules.append (rule);

    multiLineCommentFormat.setForeground (Qt::red);

    quotationFormat.setForeground (Qt::darkGreen);
    rule.pattern = QRegularExpression ("\".*\"");
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

void LuaHighlighter::highlightBlock (const QString& text)
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
