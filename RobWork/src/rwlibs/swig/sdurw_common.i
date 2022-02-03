%module sdurw_common

%include <stl.i>
%include <std_vector.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/ext_i/std.i>


%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
%}

//types.hpp
/*
typedef signed char __int8_t;
typedef unsigned char __uint8_t;
typedef signed short int __int16_t;
typedef unsigned short int __uint16_t;
typedef signed int __int32_t;
typedef unsigned int __uint32_t;
#if __WORDSIZE == 64
typedef signed long int __int64_t;
typedef unsigned long int __uint64_t;
*/

%{
    #include <rw/common/Archive.hpp>
%}
%include <rw/common/Archive.hpp>

%{
    #include <rw/common/InputArchive.hpp>
%}
%include <rw/common/InputArchive.hpp>

%{
    #include <rw/common/OutputArchive.hpp>
%}
%include <rw/common/OutputArchive.hpp>

%{
    #include <rw/common/BINArchive.hpp>
%}
%include <rw/common/BINArchive.hpp>

%{
    #include <rw/common/Cache.hpp>
%}
%include <rw/common/Cache.hpp>


#if defined(SWIGPYTHON)
RENAME(rw::common::ConcatVectorIterator::operator++,next);
RENAME(rw::common::ConstConcatVectorIterator::operator++,next);
#endif 
%{
    #include <rw/common/ConcatVectorIterator.hpp>
%}
%include <rw/common/ConcatVectorIterator.hpp>

%{
    #include <rw/common/FileCache.hpp>
%}
%include <rw/common/FileCache.hpp>

%{
    #include <rw/common/INIArchive.hpp>
%}
%include <rw/common/INIArchive.hpp>

%{
    #include <rw/common/LogBufferedChar.hpp>
%}
%include <rw/common/LogBufferedChar.hpp>

%{
    #include <rw/common/LogBufferedMsg.hpp>
%}
%include <rw/common/LogBufferedMsg.hpp>

%{
    #include <rw/common/LogFileWriter.hpp>
%}
%include <rw/common/LogFileWriter.hpp>

%{
    #include <rw/common/LogMultiWriter.hpp>
%}
%include <rw/common/LogMultiWriter.hpp>

%{
    #include <rw/common/PairMap.hpp>
%}
%include <rw/common/PairMap.hpp>

%{
    #include <rw/common/ProgramOptions.hpp>
%}
%include <rw/common/ProgramOptions.hpp>

%{
    #include <rw/common/ScopedTimer.hpp>
%}
%include <rw/common/ScopedTimer.hpp>

%{
    #include <rw/common/Serializable.hpp>
%}
%include <rw/common/Serializable.hpp>

%{
    #include <rw/common/ThreadPool.hpp>
%}
%include <rw/common/ThreadPool.hpp>
NAMED_OWNEDPTR(ThreadPool, rw::common::ThreadPool);


%{
    #include <rw/common/ThreadSafeQueue.hpp>
%}
%include <rw/common/ThreadSafeQueue.hpp>

%{
    #include <rw/common/ThreadSafeStack.hpp>
%}
%include <rw/common/ThreadSafeStack.hpp>

#if defined(SWIGPYTHON)
RENAME(rw::common::ThreadSafeVariable::operator=,set);
#endif 
%{
    #include <rw/common/ThreadSafeVariable.hpp>
%}
%include <rw/common/ThreadSafeVariable.hpp>



%{
    #include <rw/common/ThreadTask.hpp>
%}
%include <rw/common/ThreadTask.hpp>


%extend rw::common::ThreadTask{
    /**
     * @brief Get a list of exceptions registered in task and subtasks.
     * @return a list of exceptions.
     */
    std::vector<rw::core::Exception> getException() const{
        std::list<rw::core::Exception> init = $self->getExceptions();
        return std::vector<rw::core::Exception> (init.begin(),init.end());
    }
}
NAMED_OWNEDPTR(ThreadTask,rw::common::ThreadTask);
%template (VectorThreadTaskPtr) std::vector<rw::core::Ptr<rw::common::ThreadTask> >;


%{
    #include <rw/common/Timer.hpp>
%}
%include <rw/common/Timer.hpp>

%{
    #include <rw/common/TimerUtil.hpp>
%}
%include <rw/common/TimerUtil.hpp>


#if defined(SWIGPYTHON)
RENAME(rw::common::VectorIterator::operator++,next);
RENAME(rw::common::ConstVectorIterator::operator++,next);
#endif 
%{
    #include <rw/common/VectorIterator.hpp>
%}
%include <rw/common/VectorIterator.hpp>
