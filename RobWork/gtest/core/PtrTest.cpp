/******************************************************************************
 * Copyright 2020 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#include <gtest/gtest.h>

#define RW_USE_BOOST_PTR_COMPLIANCE
#include <rw/core/Ptr.hpp>

#include <boost/shared_ptr.hpp>

#include <memory>

#ifdef RW_USE_BOOST_PTR
#define TEST_NAME PtrBoostTest
#else
#define TEST_NAME PtrTest
#endif

using namespace rw::core;

namespace {
    class Monitor {
        public:
            Monitor():
                _alive(false),
                _dead(false)
            {
            }
            virtual ~Monitor() {}
            void setAlive()
            {
                _alive = true;
            }
            void setDead()
            {
                _dead = true;
            }
            bool isAlive()
            {
                return _alive;
            }
            bool isDead()
            {
                return _dead;
            }
        private:
            bool _alive;
            bool _dead;
    };

    class A {
        public:
            typedef rw::core::Ptr<A> Ptr;
            A(Monitor* const monitor = nullptr):
                _monitor(monitor),
                _isA(true),
                _isB(false)
            {
                if (_monitor != nullptr)
                    _monitor->setAlive();
            }
            virtual ~A()
            {
                if (_monitor != nullptr)
                    _monitor->setDead();
            }
            virtual void print()
            {
                std::cout << "A" << std::endl;
            }
            virtual bool isA()
            {
                return _isA;
            }
            virtual bool isB()
            {
                return _isB;
            }
        protected:
            Monitor* const _monitor;
        private:
            bool _isA;
            bool _isB;
    };

    class B: public A {
        public:
            typedef rw::core::Ptr<B> Ptr;
            B(Monitor* const monitor = nullptr):
                A(monitor),
                _isA(false),
                _isB(true)
            {
            }
            virtual void print()
            {
                std::cout << "B" << std::endl;
            }
            virtual bool isA()
            {
                return _isA;
            }
            virtual bool isB()
            {
                return _isB;
            }
        private:
            bool _isA;
            bool _isB;
    };
}

TEST(TEST_NAME, CastToSubType) {
    std::vector<A::Ptr> aptrs;
    B* const b = new B();
    aptrs.push_back( ownedPtr(b) );
    aptrs.push_back( ownedPtr(new A()) );

    std::vector<A::Ptr> aptrsCopy = aptrs;
    {
        const A::Ptr aptr = aptrsCopy[0];
        A* const a  = aptr.get();
        B* const bcast = dynamic_cast<B*>(a);
        EXPECT_TRUE(bcast != NULL);
        const B::Ptr bptr = aptr.cast<B>();
        EXPECT_TRUE(bptr != NULL);
        EXPECT_FALSE(bptr.isNull());
        EXPECT_TRUE(bptr->isB());
        const B::Ptr bptrs = aptr.scast<B>();
        EXPECT_TRUE(bptrs != NULL);
        EXPECT_FALSE(bptrs.isNull());
        EXPECT_TRUE(bptrs->isB());
    }

    {
        const A::Ptr aptr = aptrsCopy[1];
        A* const a  = aptr.get();
        B* const bcast = dynamic_cast<B*>(a);
        EXPECT_TRUE(bcast == NULL);
        const B::Ptr bptr = aptr.cast<B>();
        EXPECT_TRUE(bptr == NULL);
        EXPECT_TRUE(bptr.isNull());
        EXPECT_FALSE(aptr->isB());
        const B::Ptr bptrs = aptr.scast<B>(); // (invalid)
        EXPECT_TRUE(bptrs != NULL);
        EXPECT_FALSE(bptrs.isNull());
    }
}

TEST(TEST_NAME, ConstructFromOtherPtr) {
    A::Ptr aptr = ownedPtr(new A());
    B::Ptr bptr = ownedPtr(new B());
    A::Ptr afromb(bptr);
    //B::Ptr bfroma(aptr); This should not compile
    EXPECT_TRUE(afromb->isB());
}

TEST(TEST_NAME, OwnedTest) {
    Monitor* const monitor1 = new Monitor();
    Monitor* const monitor2 = new Monitor();
    Monitor* const monitor3 = new Monitor();
    Monitor* const monitor4 = new Monitor();
    A::Ptr ptr1 = ownedPtr(new A(monitor1));
    A* const rawptr = new A(monitor2);
    A::Ptr ptr2 = rawptr;
    boost::shared_ptr<A> bptr(new A(monitor3));
    A::Ptr ptr3 = bptr;
    std::shared_ptr<A> cptr(new A(monitor4));
    A::Ptr ptr4 = cptr;
    EXPECT_TRUE(monitor1->isAlive());
    EXPECT_TRUE(monitor2->isAlive());
    EXPECT_TRUE(monitor3->isAlive());
    EXPECT_TRUE(monitor4->isAlive());
    EXPECT_TRUE(ptr1.isShared());
    EXPECT_FALSE(ptr2.isShared());
    EXPECT_TRUE(ptr3.isShared());
    EXPECT_TRUE(ptr4.isShared());
    ptr1 = nullptr;
    EXPECT_TRUE(monitor1->isDead());
    ptr2 = nullptr;
    EXPECT_FALSE(monitor2->isDead());
    bptr = nullptr;
    EXPECT_FALSE(monitor3->isDead());
    ptr3 = nullptr;
    EXPECT_TRUE(monitor3->isDead());
    cptr = nullptr;
    EXPECT_FALSE(monitor4->isDead());
    ptr4 = nullptr;
    EXPECT_TRUE(monitor4->isDead());
    delete rawptr;
    delete monitor1;
    delete monitor2;
    delete monitor3;
    delete monitor4;
}

TEST(TEST_NAME, GetCppSharedPtr) {
    Monitor* const monitor = new Monitor();
    A::Ptr ptr = ownedPtr(new A(monitor));
    std::shared_ptr<A> cptr = ptr.getCppSharedPtr();
    EXPECT_FALSE(monitor->isDead());
    cptr = nullptr;
    EXPECT_FALSE(monitor->isDead());
    cptr = ptr;
    ptr = nullptr;
    EXPECT_FALSE(monitor->isDead());
    cptr = nullptr;
    EXPECT_TRUE(monitor->isDead());
    delete monitor;
}

TEST(TEST_NAME, GetBoostSharedPtr) {
    Monitor* const monitor = new Monitor();
    A::Ptr ptr = ownedPtr(new A(monitor));
    boost::shared_ptr<A> bptr = ptr.getBoostSharedPtr();
    EXPECT_FALSE(monitor->isDead());
    bptr = nullptr;
    EXPECT_FALSE(monitor->isDead());
    bptr = ptr;
    ptr = nullptr;
    EXPECT_FALSE(monitor->isDead());
    bptr = nullptr;
    EXPECT_TRUE(monitor->isDead());
    delete monitor;
}

TEST(TEST_NAME, NullPtr) {
    A::Ptr ptr1 = nullptr;
    A::Ptr ptr2;
    A::Ptr ptr3 = ownedPtr(new A());
    EXPECT_TRUE(ptr1.get() == nullptr);
    EXPECT_TRUE(ptr2.get() == nullptr);
    EXPECT_FALSE(ptr3.get() == nullptr);
    EXPECT_FALSE(ptr1);
    EXPECT_FALSE(ptr2);
    EXPECT_TRUE(ptr3);
    EXPECT_TRUE(ptr1.isNull());
    EXPECT_TRUE(ptr2.isNull());
    EXPECT_FALSE(ptr3.isNull());
}

TEST(TEST_NAME, Dereference) {
    A* const rawA = new A();
    B* const rawB = new B();
    A::Ptr ptrA = ownedPtr(rawA);
    B::Ptr ptrB = ownedPtr(rawB);
    // operator->
    EXPECT_TRUE(ptrA->isA());
    EXPECT_FALSE(ptrA->isB());
    EXPECT_FALSE(ptrB->isA());
    EXPECT_TRUE(ptrB->isB());
    // operator*
    EXPECT_TRUE((*ptrA).isA());
    EXPECT_FALSE((*ptrA).isB());
    EXPECT_FALSE((*ptrB).isA());
    EXPECT_TRUE((*ptrB).isB());
    // get()
    EXPECT_TRUE(ptrA.get()->isA());
    EXPECT_FALSE(ptrA.get()->isB());
    EXPECT_FALSE(ptrB.get()->isA());
    EXPECT_TRUE(ptrB.get()->isB());
}

TEST(TEST_NAME, Comparison) {
    A* const raw = new A();
    A::Ptr ptr = ownedPtr(raw);
    A::Ptr ptr2 = ptr;
    EXPECT_TRUE(raw == ptr);
    EXPECT_TRUE(ptr == raw);
    EXPECT_TRUE(ptr == ptr2);
}

TEST(TEST_NAME, VectorConversionToBoost) {
    Monitor* const monitor = new Monitor();
    std::vector<A::Ptr> ptrs;
    ptrs.push_back(ownedPtr(new A(monitor)));
    ptrs.push_back(ownedPtr(new A()));
    ptrs.push_back(ownedPtr(new B()));
    std::vector<boost::shared_ptr<A> > bptrs = toBoost(ptrs);
    EXPECT_FALSE(monitor->isDead());
    bptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    bptrs = toBoost(ptrs);
    ptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    bptrs.clear();
    EXPECT_TRUE(monitor->isDead());
    delete monitor;
}

TEST(TEST_NAME, VectorConversionToStd) {
    Monitor* const monitor = new Monitor();
    std::vector<A::Ptr> ptrs;
    ptrs.push_back(ownedPtr(new A(monitor)));
    ptrs.push_back(ownedPtr(new A()));
    ptrs.push_back(ownedPtr(new B()));
    std::vector<std::shared_ptr<A> > cptrs = toStd(ptrs);
    EXPECT_FALSE(monitor->isDead());
    cptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    cptrs = toStd(ptrs);
    ptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    cptrs.clear();
    EXPECT_TRUE(monitor->isDead());
    delete monitor;
}

TEST(TEST_NAME, VectorConversionFromBoost) {
    Monitor* const monitor = new Monitor();
    std::vector<boost::shared_ptr<A> > bptrs;
    bptrs.push_back(boost::shared_ptr<A>(new A(monitor)));
    bptrs.push_back(boost::shared_ptr<A>(new A()));
    bptrs.push_back(boost::shared_ptr<A>(new B()));
    std::vector<A::Ptr> ptrs = fromBoost(bptrs);
    EXPECT_FALSE(monitor->isDead());
    ptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    ptrs = fromBoost(bptrs);
    bptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    ptrs.clear();
    EXPECT_TRUE(monitor->isDead());
    delete monitor;
}

TEST(TEST_NAME, VectorConversionFromStd) {
    Monitor* const monitor = new Monitor();
    std::vector<std::shared_ptr<A> > cptrs;
    cptrs.push_back(std::shared_ptr<A>(new A(monitor)));
    cptrs.push_back(std::shared_ptr<A>(new A()));
    cptrs.push_back(std::shared_ptr<A>(new B()));
    std::vector<A::Ptr> ptrs = fromStd(cptrs);
    EXPECT_FALSE(monitor->isDead());
    ptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    ptrs = fromStd(cptrs);
    cptrs.clear();
    EXPECT_FALSE(monitor->isDead());
    ptrs.clear();
    EXPECT_TRUE(monitor->isDead());
    delete monitor;
}
