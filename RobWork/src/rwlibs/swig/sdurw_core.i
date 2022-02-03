%module sdurw_core

%pragma(java) jniclassclassmodifiers="class"
#if defined (SWIGJAVA)
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
SWIG_JAVABODY_TYPEWRAPPER(public, public, public, SWIGTYPE)
#endif

%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/std.i>
%include <rwlibs/swig/ext_i/boost.i>
%include <rwlibs/swig/ext_i/os.i>

%rename(getDeref) rw::core::Ptr::operator->;
%rename(deref) rw::core::Ptr::get;

%{
    #include <rw/core/Ptr.hpp>
%}

#if defined(SWIGPYTHON)
    %pythonprepend ownedPtr(T*) %{
    args[0].thisown = 0
    %}
#endif

%define PTR_EQ_C_PTR
    %extend {
        bool operator== ( T* p) {
            return $self->get() == p;
        }
    }
%enddef

%include <rw/core/Ptr.hpp>

#if defined(SWIGPYTHON)
    %pythonprepend rw::core::ownedPtr(T*) %{
    args[0].thisown = 0
    %}
#endif

#define NAME_PTR(x) x ## Ptr
#define NAME_CPTR(x) x ## CPtr 
#define SWIG_CORE_DEFINE(...) __VA_ARGS__
%define OWNEDPTR(ownedPtr_type)
    namespace rw { namespace core {
        #if defined(SWIGJAVA)
            %typemap (in) ownedPtr_type* %{
                jclass objcls = jenv->GetObjectClass(jarg1_);
                const jfieldID memField = jenv->GetFieldID(objcls, "swigCMemOwn", "Z");
                jenv->SetBooleanField(jarg1_, memField, (jboolean)false);
                $1 = *(std::remove_const<ownedPtr_type>::type **)&jarg1;
            %}
        #elif defined(SWIGLUA)
            %typemap (in,checkfn="SWIG_isptrtype") ownedPtr_type* %{
                if (!SWIG_IsOK(SWIG_ConvertPtr(L,$input,(void**)&$1,$descriptor,SWIG_POINTER_DISOWN))){
                    SWIG_fail_ptr("$symname",$input,$descriptor);
            }
            %}
        #endif
        %template (ownedPtr) ownedPtr<ownedPtr_type>;
        #if (defined(SWIGLUA) || defined(SWIGJAVA))
            %clear ownedPtr_type*;
        #endif
    }}
%enddef

%define NAMED_OWNEDPTR(name,ownedPtr_type)
    %template(NAME_PTR(name)) rw::core::Ptr<ownedPtr_type>;
    %template(NAME_CPTR(name)) rw::core::Ptr<ownedPtr_type const>;

    %extend rw::core::Ptr<ownedPtr_type>{
         rw::core::Ptr< const ownedPtr_type > cptr () {
             return $self->cptr();
         }
    }
    OWNEDPTR(SWIG_CORE_DEFINE(ownedPtr_type));
%enddef

%{
  #include <rw/core/os.hpp>
%}

#if !defined(WIN32)
NAMED_OWNEDPTR(VectorFloat,std::vector<float>);
NAMED_OWNEDPTR(VectorUL,std::vector<unsigned long>);
#endif 

#if defined(SWIGPYTHON)
%ignore rw::core::AnyPtr::operator=;
#endif
%{
    #include <rw/core/AnyPtr.hpp>
%}
%include <rw/core/AnyPtr.hpp>

%{
    #include <rw/core/DOMElem.hpp>
%}
%include <rw/core/DOMElem.hpp>
NAMED_OWNEDPTR(DOMElem, rw::core::DOMElem);

%{
    #include <rw/core/ExtensionPoint.hpp>
%}
%include <rw/core/ExtensionPoint.hpp>
%template (ExtensionPointDOMParser) rw::core::ExtensionPoint<rw::core::DOMParser>;

%{
    #include <rw/core/DOMParser.hpp>
%}
%include <rw/core/DOMParser.hpp>
NAMED_OWNEDPTR(DOMParser, rw::core::DOMParser);

%{
    #include <rw/core/DOMPropertyMapFormat.hpp>
%}
%include <rw/core/DOMPropertyMapFormat.hpp>

%{
    #include <rw/core/Event.hpp>
%}
%include <rw/core/Event.hpp>

#if defined(SWIGJAVA)
    RENAME(rw::core::Exception, RWException)
#endif

%{
    #include <rw/core/Exception.hpp>
%}
%include <rw/core/Exception.hpp>

%{
    #include <rw/core/Extension.hpp>
%}
%include <rw/core/Extension.hpp>
NAMED_OWNEDPTR(Extension, rw::core::Extension);
%template(VectorExtensionPtr) std::vector<rw::core::Ptr<rw::core::Extension>>;

%{
    #include <rw/core/ExtensionRegistry.hpp>
%}
%include <rw/core/ExtensionRegistry.hpp>

%{
    #include <rw/core/IOUtil.hpp>
%}
%include <rw/core/IOUtil.hpp>

%{
    #include <rw/core/Log.hpp>
%}
%include <rw/core/Log.hpp>
NAMED_OWNEDPTR(Log, rw::core::Log);

%extend rw::core::Log{
    /**
     * @brief Returns the LogWriter that is associated with LogIndex \b id
     *
     * If the \b id is unknown an exception is thrown.
     *
     * @param id [in] loglevel
     * @return Reference to LogWriter object
     */
    rw::core::LogWriter& getLogWriter(LogIndex id) {
        return $self->get(id);
    }
};

%{
    #include <rw/core/LogWriter.hpp>
%}
%include <rw/core/LogWriter.hpp>
NAMED_OWNEDPTR(LogWriter, rw::core::LogWriter);

#if !defined(SWIGLUA) && !defined(SWIGPYTHON)
%{
    #include <rw/core/LogStreamWriter.hpp>
%}
%include <rw/core/LogStreamWriter.hpp>
#endif 

%{
    #include <rw/core/Message.hpp>
%}
%include <rw/core/Message.hpp>

%{
    #include <rw/core/os.hpp>
%}
%include <rw/core/os.hpp>

%{
    #include <rw/core/Plugin.hpp>
%}
%include <rw/core/Plugin.hpp>
NAMED_OWNEDPTR(Plugin, rw::core::Plugin);

%{
    #include <rw/core/PropertyValue.hpp>
%}
%include <rw/core/PropertyValue.hpp>

%{
    #include <rw/core/PropertyValueBase.hpp>
%}
%include <rw/core/PropertyValueBase.hpp>
%template(VectorPropertyValueBasePtr) std::vector<rw::core::Ptr<rw::core::PropertyValueBase> >;

NAMED_OWNEDPTR(PropertyValueBase, rw::core::PropertyValueBase);

%{
    #include <rw/core/Property.hpp>
%}
%include <rw/core/Property.hpp>

%{
    #include <rw/core/PropertyBase.hpp>
%}
%include <rw/core/PropertyBase.hpp>
NAMED_OWNEDPTR(PropertyBase, rw::core::PropertyBase);

%{
    #include <rw/core/PropertyMap.hpp>
%}
%include <rw/core/PropertyMap.hpp>
%template(getBool) rw::core::PropertyMap::get<bool>;
%template(getInt) rw::core::PropertyMap::get<int>;
%template(getFloat) rw::core::PropertyMap::get<float>;
%template(getDouble) rw::core::PropertyMap::get<double>;
#if defined(SWIGPYTHON)
%template(getString) rw::core::PropertyMap::get<std::string>;
#else
%extend rw::core::PropertyMap {
    std::string getString(const std::string& id){ return $self->get<std::string>(id); }
}
#endif
%template(getVectorString) rw::core::PropertyMap::get<std::vector<std::string>>;
%template(getVectorBool) rw::core::PropertyMap::get<std::vector<bool>>;
%template(getVectorInt) rw::core::PropertyMap::get<std::vector<int>>;
%template(getVectorFloat) rw::core::PropertyMap::get<std::vector<float>>;
%template(getVectorDouble) rw::core::PropertyMap::get<std::vector<double>>;
%template(setBool) rw::core::PropertyMap::set<bool>;
%template(setInt) rw::core::PropertyMap::set<int>;
%template(setFloat) rw::core::PropertyMap::set<float>;
%template(setDouble) rw::core::PropertyMap::set<double>;
%template(setString) rw::core::PropertyMap::set<std::string>;
%template(setVectorString) rw::core::PropertyMap::set<std::vector<std::string>>;
%template(setVectorBool) rw::core::PropertyMap::set<std::vector<bool>>;
%template(setVectorInt) rw::core::PropertyMap::set<std::vector<int>>;
%template(setVectorFloat) rw::core::PropertyMap::set<std::vector<float>>;
%template(setVectorDouble) rw::core::PropertyMap::set<std::vector<double>>;

%{
    #include <rw/core/PropertyType.hpp>
%}
%include <rw/core/PropertyType.hpp>

%{
    #include <rw/core/RobWork.hpp>
%}
%include <rw/core/RobWork.hpp>

NAMED_OWNEDPTR(RobWork, rw::core::RobWork);

%{
    #include <rw/core/StringUtil.hpp>
%}
%include <rw/core/StringUtil.hpp>

/********************************************
 * LUA functions
 ********************************************/


#if defined (SWIGLUA)
%luacode {

-- Group: Lua functions
-- Var: print_to_log
print_to_log = true

-- Var: overrides the global print function
local global_print = print

-- Function: print
--  Forwards the global print functions to the sdurw.print functions
--  whenever print_to_log is defined.
function print(...)
    if print_to_log then
        for i, v in ipairs{...} do
            if i > 1 then rw.writelog("\t") end
            sdurw.writelog(tostring(v))
        end
        sdurw.writelog('\n')
    else
        global_print(...)
    end
end

-- Function:
function reflect( mytableArg )
 local mytable
 if not mytableArg then
  mytable = _G
 else
  mytable = mytableArg
 end
   local a = {} -- all functions
   local b = {} -- all Objects/Tables

 if type(mytable)=="userdata" then
   -- this is a SWIG generated user data, show functions and stuff
   local m = getmetatable( mytable )
   for key,value in pairs( m['.fn'] ) do
      if (key:sub(0, 2)=="__") or (key:sub(0, 1)==".") then
          table.insert(b, key)
      else
          table.insert(a, key)
      end
   end
   table.sort(a)
   table.sort(b)
   print("Object type: \n  " .. m['.type'])

   print("Member Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   for i,n in ipairs(b) do print("  " .. n .. "(...)") end

 else
   local c = {} -- all constants
   for key,value in pairs( mytable ) do
      -- print(type(value))
      if (type(value)=="function") then
          table.insert(a, key)
      elseif (type(value)=="number") then
          table.insert(c, key)
      else
          table.insert(b, key)
      end
   end
   table.sort(a)
   table.sort(b)
   table.sort(c)
   print("Object type: \n  " .. "Table")

   print("Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   print("Constants:")
   for i,n in ipairs(c) do print("  " .. n) end
   print("Misc:")
   for i,n in ipairs(b) do print("  " .. n) end


--  print("Metatable:")
--  for key,value in pairs( getmetatable(mytable) ) do
--      print(key);
--      print(value);
--  end

 end
 end

function help( mytable )
   reflect( mytable )
end

local used_ns = {}
local get_ns = {}

function using(ns)
  local ns_found = false
  local ns_name;
  local ns_val;
  for n,v in pairs(_G) do
    if n == ns then
      ns_found = true
      ns_name = n
      ns_val = v
      break
    end
  end
  if not ns_found then
    error("Unknown table: " .. ns)
  else
    if used_ns[ns_name] == nil then
      used_ns[ns_name] = ns_val
      for n,v in pairs(ns_val) do
        if n ~= "string" and n ~= "ownedPtr" and n ~= "doubleArray" and n ~= "doubleArray_frompointer" then
          if _G[n] ~= nil then
            print("name clash: " .. n .. " is already bound to " .. get_ns[n] .. ".".. n .. ". Binding to " .. ns .. "." .. n .. " ignored")
          else
            _G[n] = v
            get_ns[n] = ns
          end
        end
      end
    end
  end
end

function ownedPtr(arg)
  local found = false
  for ns_n,ns_v in pairs(used_ns) do
    for n,v in pairs(ns_v) do
      if type(v) ~= "function" and type(v) ~= "number" then
        if string.len(n) >= 4 then
          if string.sub(n, -3) == "Ptr" then
            if getmetatable(arg)[".type"] .. "Ptr" == n then
              return ns_v.ownedPtr(arg)
            end
          end
        end
      end
    end
  end
end

function ownedCPtr(arg)
  local found = false
  for ns_n,ns_v in pairs(used_ns) do
    for n,v in pairs(ns_v) do
      if type(v) ~= "function" and type(v) ~= "number" then
        if string.len(n) >= 5 then
          if string.sub(n, -4) == "CPtr" then
            if getmetatable(arg)[".type"] .. "CPtr" == n then
              return ns_v.ownedCPtr(arg)
            end
          end
        end
      end
    end
  end
end
}
#endif