.. _workcell-editor:

*******************
WorkCell Editor
*******************

The editor provides help to the user when making and editing the WorkCell XML format.
It does so by providing functions for setting up finished elements, syntax highlighting and auto completion.

The layout of the workcell editor is structured so that at the top, the buttons for loading, closing,
refreshing, and saving the workcell document can be accessed.
It also has a separate button for taking the current document and loading in to RobWorkStudio.

On the Second row buttons have been added for adding complete elements to the workcell file is placed.
When pressing these buttons a popup window will appear, where all necessary information can be filed in,
before returning the formated XML code for the element.

When working on multiple xml files at a time multiple tabs can be seen in the workcell editor.
Thesis can be used to navigate between multiple workcells. Using the load workcell btn the tabs
can also be used for quick switching between multiple workcells

Writing code
============
While writing code the syntax highlighter will automatically highlight most recognized arguments.
Though be aware that not all accepted keywords is highlighted yet. If ever in doubt press **ctrl+e** and a
list of accepted keywords will be displayed.

Instead of pressing add Frame the shortcut **ctrl+f** and the shortcut for add Drawable is **ctrl+d**.
Written code can like wise be saved with **ctrl+s**