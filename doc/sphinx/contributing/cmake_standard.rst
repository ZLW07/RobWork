CMake coding Standard
=====================

In order to make RobWork more streamlined, maintainable and avoid errors, follow the standard below


1. General Formating Rules
**************************

**1.1. Lowercase cmake commands**

All native CMake Commands must be in lower case.

.. code-block:: cmake

   #YES
   find_package(foo REQUIRED)
   #NO
   FIND_PACKAGE(foo REQUIRED)


**1.2. Lowercase macros**

All RobWork Macros must be in all lowercase, and named appropriately with rw, rws, rwhw or rwsim infront of the macro.

**1.3 Uppercase variables**

All variables must be in all uppercase

**1.4. Indentation**

Indent all code correctly, i.e. the body of

* if/else/endif
* foreach/endforeach
* while/endwhile
* macro/endmacro
* function/endfunction

Use 4 spaces for indenting

**1.5. End Commands**

To make the code easier to read, use empty commands for endforeach(), endif(), endfunction(), endmacro() and endwhile(). Also, use empty else() commands.

For example, do this:

.. code-block:: cmake

   if(FOOVAR)
      some_command(...)
   else()
      another_command(...)
   endif()

and not this:

.. code-block:: cmake

   if(BARVAR)
      some_other_command(...)
   else(BARVAR)
      another_command(...)
   endif(BARVAR)


**1.6. Keep lists sorted**

Whenever using a list of items where the order doesn't matter (i.e. in find_package(COMPONENTS ...), or files which should be build or installed) keep them alphabetically sorted. 
This improves readability when looking for specific items. (There are exceptions which require a specific custom order like the list of projects inside a stack).

2. Conditions and Variables
***************************

**2.1 Always quote variable that represent a string:**

.. code-block:: cmake

   set(myvar "foo")
   if("${myvar}" STREQUAL "bar")
   # ...
   endif()

**2.2 Do not quote variable that are booleans:**

.. code-block:: cmake

   set(mybvar ON)
   set(mybvar OFF)
   if(${myvar})
   # ...
   endif()

**2.3 Use if(DEFINED varname) to check if a variable is set:**

.. code-block:: cmake

   if(DEFINED myvar)
   #  ...
   endif()

**2.4 Use if(varname) to check it a variable has a non-empty value:**

.. code-block:: cmake

   if(myvar)
   #  ...
   endif()

**2.5 Path Variables**

When storing paths in variables or CACHE, do NOT have the cmake variables end up with a slash:

.. code-block:: cmake

   # YES:
   set(MY_PATH "path/to/foo")
   set(MY_OTHER_PATH "${_my_path}/${_my_var}")
   # NO:
   set(MY_PATH "path/to/foo/")
   set(MY_OTHER_PATH "${_my_path}${_my_var}")   # wrong: this is ugly

**2.6 Path Names**

Any CACHE Variable that stores a path, should be given the type PATH or FILEPATH

3. Robwork CMake setup
**********************

**3.1 Module naming**

Modules are named with prefix sdu, ie. **sdurw**, **sdurws**, **sdurwhw** and **sdurwsim**. sub-modules are named module\_"nameOfSubModule".

**3.2 File header for targets**

When a CMake file is compiling a sub-module the CMake file must include the following lines in the beginning of the file

.. code-block:: cmake

   SET(SUBSYS_NAME module_name )
   SET(SUBSYS_DESC "A description of what this module is used for" )
   SET(SUBSYS_DEPS dependency1 dependency2 ... )

**3.5 Naming of project Variables and Macros**

The Name of a project wide variable or macro must be named with the acronym of the project as prefix

.. code-block:: cmake

   #YES
   set(RWS_NAME_OF_VARIABLE ...)
   #NO
   set(NAME_OF_VARIABLE ...)
