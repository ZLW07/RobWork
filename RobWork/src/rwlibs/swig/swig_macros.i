
#if !defined(SWIGJAVA)
%include "carrays.i"
%array_class(double, doubleArray);
#else
%include "arrays_java.i";
#endif

#if defined(SWIGJAVA)
	%rename(multiply) operator*;
	%rename(divide) operator/;
	%rename(equals) operator==;
	%rename(negate) operator-() const;
	%rename(subtract) operator-;
	%rename(add) operator+;
    %rename(leessThan) operator<;
    %rename(finalizeC) finalize;

#endif

%rename(incement) operator++;

#if defined(SWIGJAVA) || defined(SWIGLUA)
    %rename(addAssign) operator +=;
    %rename(subtractAssign) operator -=;
    %rename(multiplyAssign) operator *=;
    %rename(devideAssign) operator /=;
    %rename(notEqual) operator!=;
    %rename(assign) operator=;
    %rename(streamOperator) operator<<;
#endif

#if defined(SWIGPYTHON)
    %rename(copy) operator=;
#endif 

#if defined(SWIGLUA)
    %rename(c_end) end;
#endif

#if (defined(SWIGPYTHON) || defined(SWIGLUA))
%feature("flatnested");
#endif

%{
    #include <string>
    #include <vector>
    #include <sstream>
    /**
     * @brief Convert an entity to string.
     *
     * It must be possible to stream the entity, \b x, to a ostringstream.
     * The entity should in general have a stream operator implemented.
     *
     * @param x [in] the entity to convert to string.
     * @return the string representation.
     */
    template< typename T > std::string toString (const T& x)
    {
        std::ostringstream buf;
        buf << x;
        return buf.str ();
    }

    /**
     * @brief Convert a vector of entities to a C string.
     *
     * This function uses the toString function to do the conversion.
     *
     * @param x [in] the entities to convert.
     * @return a C string representation.
     */
    template< typename T > char* printCString (const std::vector< T >& x)
    {
        static char tmp[2048];
        int idx = sprintf (tmp, "[");
        for (size_t i = 0; i < x.size (); i++)
            idx += sprintf (&tmp[idx], "%s,", toString< T > (x[i]).c_str ());
        sprintf (&tmp[idx], "]");
        return tmp;
    }

    /**
     * @brief Convert an entity to a C string.
     *
     * This function uses the toString function to do the conversion.
     *
     * @param x [in] the entity to convert.
     * @return a C string representation.
     */
    template< typename T > char* printCString (const T& x)
    {
        static char tmp[256];
        sprintf (tmp, "%s", toString< T > (x).c_str ());
        return tmp;
    }
%}

%define TOSTRING(type)
    %extend {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            char *__str__() { return printCString<type>(*$self); }
        #elif defined(SWIGJAVA)
            std::string toString() const { return toString<type>(*$self); }
        #endif
    }
%enddef

%define TOSTRING_OUTOFCLASSDEF(type)
    %extend type {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            char *__str__() { return printCString<type>(*$self); }
        #elif defined(SWIGJAVA)
            std::string toString() const { return toString<type>(*$self); }
        #endif
    }
%enddef

%define ARRAYOPERATOR(ret) // operator[]
    %extend {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            ret __getitem__(int i)const {return (*$self)[i]; }
            void __setitem__(int i,ret d){ (*$self)[i] = d; }
        #elif defined(SWIGJAVA)
            ret get(std::size_t i) const { return (*$self)[i]; }
            void set(std::size_t i,ret d){ (*$self)[i] = d; }
        #endif
    }
%enddef

%define ARRAYOPERATOR2(ret) // operator()
    %extend {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            ret __getitem__(int i)const {return (*$self)(i); }
            void __setitem__(int i,ret d){ (*$self)(i) = d; }
        #elif defined(SWIGJAVA)
            ret get(std::size_t i) const { return (*$self)(i); }
            void set(std::size_t i,ret d){ (*$self)(i) = d; }
        #endif
    }
%enddef

%define MATRIXOPERATOR(ret)
    %extend {
        #if defined(SWIGPYTHON)
            /*This typemap makes it possible to access Matrix class elements using following syntax:
            
            myMatrix[1, 1] = value
            print myMatrix[1, 1]
            
            -- using __getitem__ and __setitem__ methods. */
            %typemap(in) int[2](int temp[2]) {
                if (PyTuple_Check($input)) {
                    if (!PyArg_ParseTuple($input, "ii", temp, temp+1)) {
                        PyErr_SetString(PyExc_TypeError, "tuple must have 2 elements");
                        return NULL;
                    }
                    $1 = &temp[0];
                } else {
                    PyErr_SetString(PyExc_TypeError, "expected a tuple.");
                    return NULL;
                }
            }
            
            ret __getitem__(int i[2])const {return (*$self)(i[0], i[1]); }
            void __setitem__(int i[2], ret d){ (*$self)(i[0], i[1]) = d; }
        #elif defined(SWIGLUA)
            ret __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row, column); }
            void __setitem__(std::size_t row, std::size_t column,ret d){ (*$self)(row, column) = d; }
        #elif defined(SWIGJAVA)
            ret get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
            void set(std::size_t row, std::size_t column, ret d){ (*$self)(row, column) = d; }
        #endif
    }
%enddef

%define MAPOPERATOR(ret,index)
    %extend {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            ret __getitem__(index i)const {return (*$self)[i]; }
            void __setitem__(index i,ret d){ (*$self)[i] = d; }
        #elif defined(SWIGJAVA)
            ret get(index i) const { return (*$self)[i]; }
            void set(index i,ret d){ (*$self)[i] = d; }
        #endif
    }
%enddef

%define MAPOPERATOR2(ret,index)
    %extend {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            ret __getitem__(index i)const {return (*$self)(i); }
            void __setitem__(index i,ret d){ (*$self)(i) = d; }
        #elif defined(SWIGJAVA)
            ret get(index i) const { return (*$self)(i); }
            void set(index i,ret d){ (*$self)(i) = d; }
        #endif
    }
%enddef

%define MAP2OPERATOR(ret, i1,i2)
    %extend {
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            ret __getitem__(i1 row, i2 column)const {return (*$self)(row, column); }
            void __setitem__(i1 row, i2,ret d){ (*$self)(row, column) = d; }
        #elif defined(SWIGJAVA)
            ret get(i1 row, i2 column) const { return (*$self)(row, column); }
            void set(i1 row, i2 column, ret d){ (*$self)(row, column) = d; }
        #endif
    }
%enddef

%define CALLOPERATOR(ret, input_type)
    %extend {
        #if defined(SWIGLUA)
            ret __call(input_type in) { return (*$self)(in);}
        #elif defined(SWIGPYTHON)
            ret __call__(input_type in) { return (*$self)(in);}
        #elif defined(SWIGJAVA)
            ret call(input_type in) { return (*$self)(in);}
        #endif
    }
%enddef


%define FRIEND_OPERATOR(friend_class, orig_class, theOprator)
    %extend friend_class {
        orig_class operator theOprator (const orig_class& rhs) {
            return (*$self) theOprator rhs;
        }
    }
%enddef

%define FRIEND_OPERATOR_RET(friend_class, orig_class, theOprator, ret)
    %extend friend_class {
        ret operator theOprator (const orig_class& rhs) {
            return (*$self) theOprator rhs;
        }
    }
%enddef

%define INCREMENT(ret)
    %extend {
        /**
         * @brief Increments the class
         * @return The incremented class
         */
        ret increment() {return *$self++; }
    }
%enddef

%define COPY(ret)
    %extend {
        /**
         * @brief Creates a copy of the class
         * @return a copy of the class
         */
        ret copy() {return *$self; }
    }
%enddef

%define RENAME(from, to)
    %rename (to) from;
%enddef

%define SWIG_DECLARE_TEMPLATE(name,type)
    %template(name) type;
%enddef

%define IMPORT_FILE(include_module, file)
    %import(module=include_module) file
%enddef

%define IMPORT_MODULE(include_module)
    %import include_module
%enddef

%define ADD_DEFINITION(orig,new)
#if defined(SWIGPYTHON)
%pythoncode {
new=orig
}
#elif defined(SWIGLUA)
%luacode {
new = orig
}
#elif defined(SWIGJAVA)
#endif 
%enddef