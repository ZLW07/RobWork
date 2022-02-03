#include <rw/core/Ptr.hpp>

using namespace rw::common;

class T {
    public:
        typedef rw::common::Ptr<T> Ptr;
        typedef rw::common::Ptr<const T> CPtr;

        static Ptr make()
        {
            return ownedPtr(new T);
        }
};
