%module sdurw_kinematics

%include <rwlibs/swig/swig_macros.i>

%include <stl.i>
%include <std_vector.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>

%import <rwlibs/swig/ext_i/std.i>
%{
    #include <rw/common/BINArchive.hpp>
    #include <rw/common/INIArchive.hpp>
%}


%pragma(java) jniclassimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
%}
%pragma(java) moduleimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
%}
%typemap(javaimports) SWIGTYPE %{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
%}



%ignore rw::kinematics::StateData::getData;
%ignore rw::kinematics::StateData::setData;
%ignore rw::kinematics::StateData::getCache() const;
%ignore rw::kinematics::StateData::getCache(rw::kinematics::State &);
%{
    #include <rw/kinematics/StateData.hpp>
%}
%include <rw/kinematics/StateData.hpp>
%extend rw::kinematics::StateData{
    /**
     * @brief An array of length size() containing the values for
     * the state data.
     *
     * It is OK to call this method also for a StateData with zero size.
     *
     * @param state [in] The state containing the StateData values.
     *
     * @return The values for the frame.
     */
    std::vector<double> getData(rw::kinematics::State& state){
        double* data = $self->getData(state);
        std::vector<double> ret($self->size());
        for(int i = 0; i < $self->size(); i++){
            ret.push_back(data[i]);
        }
        return ret;
    }
    /**
     * @brief Assign for \b state data the size() of values of the array \b
     * vals.
     *
     * The array \b vals must be of length at least size().
     *
     * @param state [inout] The state to which \b vals are written.
     *
     * @param vals [in] The joint values to assign.
     *
     * setData() and getData() are related as follows:
     * \code
     * data.setData(state, q_in);
     * const double* q_out = data.getData(state);
     * for (int i = 0; i < data.getDOF(); i++)
     *   q_in[i] == q_out[i];
     * \endcode
     */
    void setData(rw::kinematics::State& state, const std::vector<double> vals){
        $self->setData(state,vals.data());
    }
}
%template(VectorStateDataPtr) std::vector<rw::core::Ptr<rw::kinematics::StateData>>;
NAMED_OWNEDPTR(StateData, rw::kinematics::StateData);


%ignore rw::kinematics::Frame::getPropertyMap() const;

%ignore rw::kinematics::Frame::getDafParent(rw::kinematics::State const &) const;

%ignore rw::kinematics::Frame::getDafChildren(rw::kinematics::State const &) const;
#if defined(SWIGJAVA)
%ignore rw::kinematics::Frame::getChildren();
%ignore rw::kinematics::Frame::getParent();
%ignore rw::kinematics::Frame::getParent(rw::kinematics::State const &) const;
%ignore rw::kinematics::Frame::getChildren(rw::kinematics::State const &) const;
#else 
%ignore rw::kinematics::Frame::getChildren() const;
%ignore rw::kinematics::Frame::getParent() const;

#endif 
%{
    #include <rw/kinematics/Frame.hpp>
%}
%include <rw/kinematics/Frame.hpp>

NAMED_OWNEDPTR(Frame, rw::kinematics::Frame);

%template (FrameVector) std::vector<rw::kinematics::Frame*>;
%template (FramePair) std::pair< rw::kinematics::Frame *, rw::kinematics::Frame * >;
%template (FramePairVector) std::vector< std::pair< rw::kinematics::Frame *, rw::kinematics::Frame * > >;
%template (VectorVectorFrame) std::vector<std::vector<rw::kinematics::Frame*>>;
%template (MapStringFrame) std::map<std::string,rw::kinematics::Frame*>;

%{
    #include <rw/kinematics/FixedFrame.hpp>
%}
%include <rw/kinematics/FixedFrame.hpp>
%template(VectorFixedFrame) std::vector<rw::kinematics::FixedFrame*>;
NAMED_OWNEDPTR(FixedFrame, rw::kinematics::FixedFrame);

%{
    #include <rw/kinematics/FKRange.hpp>
%}
%include <rw/kinematics/FKRange.hpp>
NAMED_OWNEDPTR(FKRange,rw::kinematics::FKRange);

%ignore rw::kinematics::FKTable::get(const rw::kinematics::Frame&) const;
%{
    #include <rw/kinematics/FKTable.hpp>
%}
%include <rw/kinematics/FKTable.hpp>
NAMED_OWNEDPTR(FKTable,rw::kinematics::FKTable);

%{
    #include <rw/kinematics/FrameMap.hpp>
%}
%include <rw/kinematics/FrameMap.hpp>
//NAMED_OWNEDPTR(FrameMap,rw::kinematics::FrameMap);

%{
    #include <rw/kinematics/FramePairMap.hpp>
%}
%include <rw/kinematics/FramePairMap.hpp>
//NAMED_OWNEDPTR(FramePairMap,rw::kinematics::FramePairMap);

%{
    #include <rw/kinematics/FrameType.hpp>
%}
%include <rw/kinematics/FrameType.hpp>
NAMED_OWNEDPTR(FrameType,rw::kinematics::FrameType);

%nodefaultctor Kinematics;
%{
    #include <rw/kinematics/Kinematics.hpp>
%}
%include <rw/kinematics/Kinematics.hpp>
NAMED_OWNEDPTR(Kinematics,rw::kinematics::Kinematics);

%{
    #include <rw/kinematics/MovableFrame.hpp>
%}
%include <rw/kinematics/MovableFrame.hpp>
%template (MovableFrameVector) std::vector<rw::kinematics::MovableFrame *> ;
NAMED_OWNEDPTR(MovableFrame, rw::kinematics::MovableFrame);

%ignore rw::kinematics::QState::getQ;
%ignore rw::kinematics::QState::setQ;
%{
    #include <rw/kinematics/QState.hpp>
%}
%include <rw/kinematics/QState.hpp>
NAMED_OWNEDPTR(QState,rw::kinematics::QState);
%extend rw::kinematics::QState{
    /**
     * @brief An array of length frame.getDOF() containing the joint values
     * for \b frame.
     *
     * It is OK to call this method also for frames with zero degrees of
     * freedom.
     *
     * @return The joint values for the frame.
     */
    std::vector<double> getQ(const rw::kinematics::StateData& SData){
        double* data = $self->getQ(SData);
        std::vector<double> ret($self->size());
        for(int i = 0; i < $self->size(); i++){
            ret.push_back(data[i]);
        }
        return ret;
    }
    
    /**
     * @brief Assign for \b frame the frame.getDOF() joint values of the
     * array \b vals.
     *
     * The array \b vals must be of length at least frame.getDOF().
     *
     * @param data [in] The StateData for which the joint values are assigned.
     *
     * @param vals [in] The joint values to assign.
     *
     * setQ() and getQ() are related as follows:
     * \code
     * q_state.setQ(frame, q_in);
     * const double* q_out = q_state.getQ(frame);
     * for (int i = 0; i < frame.getDOF(); i++)
     *   q_in[i] == q_out[i];
     * \endcode
     */
    void setQ(const rw::kinematics::StateData& data, const std::vector<double> vals){
        $self->setQ(data,vals.data());
    }
}

%{
    #include <rw/kinematics/State.hpp>
%}
%include <rw/kinematics/State.hpp>
//%template (VectorState) std::vector<rw::kinematics::State>;

%{
    #include <rw/kinematics/StateCache.hpp>
%}
%include <rw/kinematics/StateCache.hpp>
NAMED_OWNEDPTR(StateCache, rw::kinematics::StateCache);

%ignore rw::kinematics::Stateless::getStateStructure() const;
%{
    #include <rw/kinematics/Stateless.hpp>
%}
%include <rw/kinematics/Stateless.hpp>
//NAMED_OWNEDPTR(Stateless,rw::kinematics::Stateless);

%{
    #include <rw/kinematics/StatelessData.hpp>
%}
%include <rw/kinematics/StatelessData.hpp>
//NAMED_OWNEDPTR(StatelessData,rw::kinematics::StatelessData);

%ignore rw::kinematics::StateSetup::getFrame(int) const;
%ignore rw::kinematics::StateSetup::getTree() const;
%{
    #include <rw/kinematics/StateSetup.hpp>
%}
%include <rw/kinematics/StateSetup.hpp>
NAMED_OWNEDPTR(StateSetup, rw::kinematics::StateSetup);

%ignore rw::kinematics::StateStructure::getRoot() const;
%{
    #include <rw/kinematics/StateStructure.hpp>
%}
%include <rw/kinematics/StateStructure.hpp>
NAMED_OWNEDPTR(StateStructure, rw::kinematics::StateStructure);

%ignore rw::kinematics::TreeState::getParent(rw::kinematics::Frame const *) const;
%{
    #include <rw/kinematics/TreeState.hpp>
%}
%include <rw/kinematics/TreeState.hpp>
NAMED_OWNEDPTR(TreeState,rw::kinematics::TreeState);
