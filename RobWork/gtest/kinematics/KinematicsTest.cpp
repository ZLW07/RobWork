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

#include <gtest/gtest.h>

#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/Stateless.hpp>
#include <rw/kinematics/StatelessData.hpp>

#include <memory>
//

using namespace rw::kinematics;
using namespace rw::math;
/*
void sharedPtrTest(){
    BOOST_TEST_MESSAGE("Shared PTR test");
    typedef std::vector<std::shared_ptr<Frame> > SharedVector;
    int N=10;
    SharedVector f1,f2;
    {
        SharedVector frames(N);
        for(int i = 0;i<N; i++){
            std::ostringstream ostr;
            ostr << "L" << i;
            frames[i] = std::shared_ptr<Frame>(new FixedFrame(ostr.str(), Transform3D<>(Vector3D<>(1,2,3))));
        }
        f1 = frames;
    }
    {
        SharedVector vf = f1;
    }
    {
        SharedVector vf = f1;
        f2 = vf;
    }
    for(std::shared_ptr<Frame>& frame: f1){
        std::cout << frame->getName() << " " << frame.use_count() << std::endl;
    }
    for(std::shared_ptr<Frame>& frame: f2){
        std::cout << frame->getName() << " " << frame.use_count() << std::endl;
    }
}
*/

class StateCacheObject: public StateCache {
public:
    int A;

    StateCacheObject():A(0){}

    size_t size() const{ return sizeof(StateCacheObject); };

    StateCache::Ptr clone() const{
        StateCacheObject *scache = new StateCacheObject();
        scache->A = A;
        return rw::core::ownedPtr( scache );
    }
};

class StateDataWithCache: public StateData {
public:
    StateDataWithCache():StateData(2,"MyStateWithCache", rw::core::ownedPtr( new StateCacheObject() ) ){
    }

    void setA(int i, State &state){

        StateCache::Ptr cache = this->getCache(state);

        // if cache is empty, then initialize it
        if(cache == NULL){

            cache = rw::core::ownedPtr( new StateCacheObject() );

            setCache( cache , state);

        }

        StateCacheObject *sobj = (StateCacheObject*)cache.get();

        sobj->A = i;

    }

    int getA(State &state){
        StateCache::Ptr cache = this->getCache(state);
        // if cache is empty, then initialize it
        if(cache == NULL){
            cache = rw::core::ownedPtr( new StateCacheObject() );
            setCache( cache , state);
        }
        StateCacheObject *sobj = (StateCacheObject*)cache.get();
        return sobj->A;
    }


};

TEST(KinematicsTest, StatelessObjectTest )
{
	struct MyObj: public Stateless {
		MyObj()
		{
			add(_ival);
			add(_v3d);
		}

		StatelessData<int> _ival;
		StatelessData<Vector3D<> > _v3d;
	};
	std::shared_ptr<StateStructure> tree( new StateStructure() );
	State state = tree->getDefaultState();

	// create stateless object
	MyObj obj;
	// test adding object to state
	obj.registerIn( state );


	obj._ival.get(state) = 25;
	obj._v3d.get(state) = Vector3D<>(0,1,2);

	EXPECT_EQ(25, obj._ival.get(state));


}

TEST(KinematicsTest, StateStructureTest )
{
    //sharedPtrTest();

    FixedFrame* l1 = new FixedFrame("l1", Transform3D<>(Vector3D<>(1,2,3)));
    MovableFrame* m1 = new MovableFrame("m1");
    FixedFrame* daf = new FixedFrame("daf", Transform3D<>(Vector3D<>(1,2,3)));

    std::shared_ptr<StateStructure> tree( new StateStructure() );
    Frame* world = tree->getRoot();
    tree->addFrame(l1,world);
    tree->addFrame(m1,world);
    tree->addDAF(daf,world);
    // Todo: check if frames are in the tree, and if parents was set correctly

    State state = tree->getDefaultState();

    Transform3D<> m1_t3d( Vector3D<>(1,2,3));
    m1->setTransform(m1_t3d, state);
    Transform3D<> m1_t3d_b;
    m1_t3d_b= m1->getTransform(state);
    EXPECT_NEAR( m1_t3d_b.P()[0], m1_t3d.P()[0] , 1e-6);
    EXPECT_NEAR( m1_t3d_b.P()[1], m1_t3d.P()[1] , 1e-6);
    EXPECT_NEAR( m1_t3d_b.P()[2], m1_t3d.P()[2] , 1e-6);

    EXPECT_EQ( world , daf->getParent(state) );
    daf->attachTo(m1, state);
    EXPECT_EQ( m1 , daf->getParent(state) );
    daf->attachTo(l1, state);
    EXPECT_EQ( l1 , daf->getParent(state) );

    tree->setDefaultState(state);
    // todo: test delete func, adding has allready been tested
    tree->remove(l1);

    // the daf should have changed its parent to world in the default state
    Frame *dafParent = daf->getParent(tree->getDefaultState());
    EXPECT_EQ(world, dafParent);
    // now add a new l1 frame. Remember the tree took ownership of the old
    // l1 frame so we are not allowed to use that again
    l1 = new FixedFrame("l1b", Transform3D<>(Vector3D<>(1,2,3)));
    tree->addFrame(l1,world);
    state = tree->upgradeState(state);
    daf->attachTo(l1, state);
    tree->setDefaultState(state);

    // todo: test copy of state func
    MovableFrame* m2 = new MovableFrame("m2");
    tree->addFrame(m2,world);
    State nstate = tree->getDefaultState();
    nstate.copy(state);

    EXPECT_EQ( l1 , daf->getParent(nstate) );
    m1_t3d_b= m1->getTransform(nstate);
    EXPECT_NEAR( m1_t3d_b.P()[0], m1_t3d.P()[0] , 1e-6);
    EXPECT_NEAR( m1_t3d_b.P()[1], m1_t3d.P()[1] , 1e-6);
    EXPECT_NEAR( m1_t3d_b.P()[2], m1_t3d.P()[2] , 1e-6);

    // we uprade the old state and check if the newly added frame is there
    state = tree->upgradeState(state);
    Transform3D<> m2_t3d( Vector3D<>(1,2,3));
    m2->setTransform(m2_t3d, state);
    Transform3D<> m2_t3d_b;
    m2_t3d_b= m2->getTransform(state);
    EXPECT_NEAR( m2_t3d_b.P()[0], m2_t3d.P()[0] , 1e-6);
    EXPECT_NEAR( m2_t3d_b.P()[1], m2_t3d.P()[1] , 1e-6);
    EXPECT_NEAR( m2_t3d_b.P()[2], m2_t3d.P()[2] , 1e-6);

    // also check if old state values are not deleted
    EXPECT_EQ( l1 , daf->getParent(state) );
    m1_t3d_b= m1->getTransform(state);
    EXPECT_NEAR( m1_t3d_b.P()[0], m1_t3d.P()[0] , 1e-6);
    EXPECT_NEAR( m1_t3d_b.P()[1], m1_t3d.P()[1] , 1e-6);
    EXPECT_NEAR( m1_t3d_b.P()[2], m1_t3d.P()[2] , 1e-6);


    // this should test the influence on StateCache when state is copied/deep-copied

    StateDataWithCache *o1 = new StateDataWithCache();

    tree->addData( o1 );

    nstate = tree->getDefaultState();

    o1->setA(1,nstate);

    // standard shallow copy
    State cstate = nstate;
    // first test that the values are the same in the two states
    EXPECT_EQ(o1->getA(nstate),o1->getA(cstate));
    // now change it in one state and verify that the values are still the same
    o1->setA(20,nstate);
    EXPECT_EQ(o1->getA(nstate),o1->getA(cstate));

    // deep copy
    State dcstate = nstate.clone();
    // first test that the values are the same in the two states
    EXPECT_EQ(o1->getA(nstate),o1->getA(dcstate));
    // now change it in one state and verify that the values are NOT the same anymore
    o1->setA(50,nstate);
    EXPECT_NE(o1->getA(nstate),o1->getA(dcstate));






    std::vector<Frame*> frames = Kinematics::findAllFrames(world,state);
}

TEST(KinematicsTest, removeFramesTest )
{

    FixedFrame* l1 = new FixedFrame("l1",Transform3D<>());
    FixedFrame* l2 = new FixedFrame("l2",Transform3D<>());
    FixedFrame* l3 = new FixedFrame("l3",Transform3D<>());
    FixedFrame* l4 = new FixedFrame("l4",Transform3D<>());
    FixedFrame* l5 = new FixedFrame("l5",Transform3D<>());
    FixedFrame* l6 = new FixedFrame("l6",Transform3D<>());
    FixedFrame* l7 = new FixedFrame("l7",Transform3D<>());

    std::shared_ptr<StateStructure> tree(new StateStructure());
    Frame* world = tree->getRoot();
    tree->addFrame(l1, world);
    tree->addFrame(l2, l1);
    tree->addFrame(l3, l2);
    tree->addFrame(l4, l3);
    tree->addFrame(l5, l4);
    tree->addFrame(l6, l5);
    tree->addFrame(l7, l6);

    State state = tree->getDefaultState();
    Frame *frame = l7;
    Frame *parent; // = frame->getParent(tree->getDefaultState());
    while(frame != world) {
        parent = frame->getParent(tree->getDefaultState());
        //std::cout << "frame: " << frame->getName() << "  parent: " << parent->getName() << std::endl;
        tree->remove(frame);
        frame = parent;
    }
}

TEST(KinematicsTest, removeMovableFramesTest )
{

	MovableFrame *l1 = new MovableFrame("l1");     
    std::shared_ptr<StateStructure> tree(new StateStructure());
    Frame* world = tree->getRoot();
    tree->addFrame(l1, world);
    
    State state = tree->getDefaultState();   
    l1->setTransform(Transform3D<>::identity(), state);
    
    tree->remove(l1);  

    EXPECT_EQ( NULL , tree->findFrame("l1") );
}

TEST(KinematicsTest, singleChainTest )
{

    FixedFrame* l1 = new FixedFrame(
        "l1",
        Transform3D<>(Vector3D<>(1, 2, 3)));
    FixedFrame* l2 = new FixedFrame(
        "l2",
        Transform3D<>(Vector3D<>(2, 3, 4)));
    FixedFrame* l3 = new FixedFrame(
        "l3",
        Transform3D<>(Vector3D<>(3, 4, 5)));

    std::shared_ptr<StateStructure> tree(new StateStructure());
    Frame* world = tree->getRoot();
    tree->addFrame(l1, world);
    tree->addFrame(l2, l1);
    tree->addFrame(l3, l2);

    State state = tree->getDefaultState();
    Transform3D<> transform = Kinematics::frameTframe(world, l3, state);

    EXPECT_EQ(transform.P()(0) , 6.0);
    EXPECT_EQ(transform.P()(1) , 9.0);
    EXPECT_EQ(transform.P()(2) , 12.0);
}

TEST(KinematicsTest, multipleChainTest )
{
    FixedFrame* l1 = new FixedFrame("l1", Transform3D<>(Vector3D<>(1,2,3)));
    FixedFrame* l2 = new FixedFrame("l2", Transform3D<>(Vector3D<>(2,3,4)));

    std::shared_ptr<StateStructure> tree( new StateStructure() );
    Frame* world = tree->getRoot();
    tree->addFrame(l1,world);
    tree->addFrame(l2,world);

    State state = tree->getDefaultState();
    Transform3D<> transform = Kinematics::frameTframe(world, l1, state);
    EXPECT_EQ(transform.P()(0) , 1.0);
    EXPECT_EQ(transform.P()(1) , 2.0);
    EXPECT_EQ(transform.P()(2) , 3.0);

    transform = Kinematics::frameTframe(world, l2, state);

    EXPECT_EQ(transform.P()(0) , 2.0);
    EXPECT_EQ(transform.P()(1) , 3.0);
    EXPECT_EQ(transform.P()(2) , 4.0);
}

