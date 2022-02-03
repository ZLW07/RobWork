#ifndef INDEXEDARRAY_HPP_
#define INDEXEDARRAY_HPP_

#if !defined(SWIG)
#include <cstddef>
#include <vector>
#endif 

template< class OBJ, class T = int > class IndexedArray
{
  private:
    const std::vector< OBJ >* _objArr;
    const std::vector< T >& _idxArr;

  public:
    IndexedArray (const std::vector< OBJ >* objArr, const std::vector< T >& idxArr) :
        _objArr (objArr), _idxArr (idxArr)
    {}

    virtual ~IndexedArray () {}

    const std::vector< T >& getIndexes () const { return _idxArr; }

#if !defined(SWIG)
    const OBJ& operator[] (T i) const { return (*_objArr)[_idxArr[i]]; }
#else 
    ARRAYOPERATOR(OBJ);
#endif

    size_t size () const { return _idxArr.size (); }
};

#endif /*INDEXEDARRAY_HPP_*/
