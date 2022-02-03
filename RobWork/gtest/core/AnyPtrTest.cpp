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

#include <rw/core/AnyPtr.hpp>

using namespace rw::core;

namespace {
    class A {
        public:
            typedef rw::core::Ptr<A> Ptr;
            A():
                _isA(true),
                _isB(false)
            {
            }
            virtual ~A() {}
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
        private:
            bool _isA;
            bool _isB;
    };

    class B: public A {
        public:
            typedef rw::core::Ptr<B> Ptr;
            B():
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

TEST(AnyPtrTest, CastToTypes) {
    std::vector<AnyPtr> anyptrs;
    B* const b = new B();
    anyptrs.push_back( ownedPtr(b) );
    anyptrs.push_back( ownedPtr(new A()) );

    std::vector<AnyPtr> anyptrsCopy = anyptrs;
    {
        AnyPtr anyptr = anyptrsCopy[0];
        A* const a  = anyptr.get<A>();
        EXPECT_TRUE(a == NULL);
        B* const b = anyptr.get<B>();
        EXPECT_TRUE(b);
        const A::Ptr aptr = anyptr.cast<A>();
        EXPECT_TRUE(aptr == NULL);
        EXPECT_TRUE(aptr.isNull());
        const B::Ptr bptr = anyptr.cast<B>();
        EXPECT_TRUE(bptr != NULL);
        EXPECT_FALSE(bptr.isNull());
        EXPECT_TRUE(bptr->isB());
    }

    {
        AnyPtr anyptr = anyptrsCopy[1];
        A* const a  = anyptr.get<A>();
        EXPECT_TRUE(a != NULL);
        EXPECT_TRUE(a->isA());
        B* const bcast = dynamic_cast<B*>(a);
        EXPECT_FALSE(bcast);
        const A::Ptr aptr = anyptr.cast<A>();
        EXPECT_TRUE(aptr != NULL);
        EXPECT_FALSE(aptr.isNull());
        EXPECT_TRUE(aptr->isA());
        const B::Ptr bptr = anyptr.cast<B>(); // (invalid)
        EXPECT_TRUE(bptr == NULL);
        EXPECT_TRUE(bptr.isNull());
    }
}
