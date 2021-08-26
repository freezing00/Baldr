////////////////////////////////////////////////////////////////////////////////
// flex_string
// Copyright (c) 2001 by Andrei Alexandrescu
// Permission to use, copy, modify, distribute and sell this software for any
//     purpose is hereby granted without fee, provided that the above copyright
//     notice appear in all copies and that both that copyright notice and this
//     permission notice appear in supporting documentation.
// The author makes no representations about the
//     suitability of this software for any purpose. It is provided "as is"
//     without express or implied warranty.
////////////////////////////////////////////////////////////////////////////////

#ifndef DAHUA_FLEX_STRING_SHELL_INC_
#define DAHUA_FLEX_STRING_SHELL_INC_

// revision 1111


///////////////////////////////////////////////////////////////////////////////
// class template flex_string
// This file does not include any storage policy headers
///////////////////////////////////////////////////////////////////////////////

#include <memory>
#include <algorithm>
#include <functional>
#include <cassert>
#include <limits>
#include <stdexcept>
#include "flex_string_details.h"
#include <string>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////

namespace Dahua {
namespace Infra {


// Forward declaration for default storage policy
template <typename E, class A> class AllocatorStringStorage;


template <class T> class mallocator
{
public:
    typedef T                 value_type;
    typedef value_type*       pointer;
    typedef const value_type* const_pointer;
    typedef value_type&       reference;
    typedef const value_type& const_reference;
    typedef size_t            size_type;
    //typedef unsigned int      size_type;
    //typedef std::ptrdiff_t    difference_type;
    typedef int               difference_type;

    template <class U>
    struct rebind { typedef mallocator<U> other; };

    mallocator() {}
    mallocator(const mallocator&) {}
    //template <class U>
    //mallocator(const mallocator<U>&) {}
    ~mallocator() {}

    pointer address(reference x) const { return &x; }
    const_pointer address(const_reference x) const
    {
        return x;
    }

    pointer allocate(size_type n, const_pointer = 0)
    {
        using namespace std;
        void* p = malloc(n * sizeof(T));
        if (!p) throw bad_alloc();
        return static_cast<pointer>(p);
    }

    void deallocate(pointer p, size_type)
    {
        using namespace std;
        free(p);
    }

    size_type max_size() const
    {
        return static_cast<size_type>(-1) / sizeof(T);
    }

    void construct(pointer p, const value_type& x)
    {
        new(p) value_type(x);
    }

    void destroy(pointer p)
    {
        p->~value_type();
    }

private:
    void operator=(const mallocator&);
};

template<> class mallocator<void>
{
  typedef void        value_type;
  typedef void*       pointer;
  typedef const void* const_pointer;

  template <class U>
  struct rebind { typedef mallocator<U> other; };
};

template <class T>
inline bool operator==(const mallocator<T>&,
                       const mallocator<T>&) {
  return true;
}

template <class T>
inline bool operator!=(const mallocator<T>&,
                       const mallocator<T>&) {
  return false;
}

template <class Allocator>
typename Allocator::pointer Reallocate(
    Allocator& alloc,
    typename Allocator::pointer p,
    typename Allocator::size_type oldObjCount,
    typename Allocator::size_type newObjCount,
    void*)
{
    // @@@ not implemented
}

template <class Allocator>
typename Allocator::pointer Reallocate(
    Allocator& alloc,
    typename Allocator::pointer p,
    typename Allocator::size_type oldObjCount,
    typename Allocator::size_type newObjCount,
    mallocator<void>*)
{
    // @@@ not implemented
}


////////////////////////////////////////////////////////////////////////////////
// class template flex_string
// a std::basic_string compatible implementation
// Uses a Storage policy
////////////////////////////////////////////////////////////////////////////////

template <typename E,
    class T = std::char_traits<E>,
    class A = std::allocator<E>,
    class Storage = AllocatorStringStorage<E, A> >
class flex_string : private Storage
{

    template <typename Exception>
    static void Enforce(bool condition, Exception*, const char* msg)
    { if (!condition) throw Exception(msg); }

    bool Sane() const
    {
        return
            begin() <= end() &&
            empty() == (size() == 0) &&
            empty() == (begin() == end()) &&
            size() <= max_size() &&
            capacity() <= max_size() &&
            size() <= capacity();
    }

    struct Invariant;
    friend struct Invariant;
    struct Invariant
    {
#ifdef DAHUA_DEBUG_FLEX_STRING
        Invariant(const flex_string& s) : s_(s)
        {
            assert(s_.Sane());
        }
        ~Invariant()
        {
            assert(s_.Sane());
        }
    private:
        const flex_string& s_;
#else
        Invariant(const flex_string&) {}
#endif
    private:
    Invariant& operator=(const Invariant&);
    };

public:
    // types
    typedef T traits_type;
    typedef typename traits_type::char_type value_type;
    typedef A allocator_type;
    typedef typename A::size_type size_type;
    typedef typename A::difference_type difference_type;

    typedef typename Storage::reference reference;
    typedef typename A::const_reference const_reference;
    typedef typename A::pointer pointer;
    typedef typename A::const_pointer const_pointer;

    typedef typename Storage::iterator iterator;
    typedef typename Storage::const_iterator const_iterator;
    typedef std::reverse_iterator<iterator
#ifdef NO_ITERATOR_TRAITS
    , value_type
#endif
    > reverse_iterator;
    typedef std::reverse_iterator<const_iterator
#ifdef NO_ITERATOR_TRAITS
    , const value_type
#endif
    > const_reverse_iterator;

    static const size_type npos;    // = size_type(-1)

private:
    static size_type Min(size_type lhs, size_type rhs)
    { return lhs < rhs ? lhs : rhs; }
    static size_type Max(size_type lhs, size_type rhs)
    { return lhs > rhs ? lhs : rhs; }
    static void Procust(size_type& n, size_type nmax)
    { if (n > nmax) n = nmax; }

public:
    // 21.3.1 construct/copy/destroy
    explicit flex_string(const A& a = A())
    : Storage(a)
    {}

    flex_string(const flex_string& str)
    : Storage(str)
    {}

    flex_string(const flex_string& str, size_type pos,
        size_type n = npos, const A& a = A())
    : Storage(a)
    {
        assign(str, pos, n);
    }

    flex_string(const value_type* s, const A& a = A())
    : Storage(s, traits_type::length(s), a)
    {}

    flex_string(const value_type* s, size_type n, const A& a = A())
    : Storage(s, n, a)
    {}

    flex_string(size_type n, value_type c, const A& a = A())
    : Storage(n, c, a)
    {}

    template <class InputIterator>
    flex_string(InputIterator begin, InputIterator end, const A& a = A())
    : Storage(a)
    {
        assign(begin, end);
    }

    ~flex_string()
    {}

    flex_string& operator=(const flex_string& str)
    {
        Storage& s = *this;
        s = str;
        return *this;
    }

    flex_string& operator=(const value_type* s)
    {
        assign(s);
        return *this;
    }

    flex_string& operator=(value_type c)
    {
        assign(1, c);
        return *this;
    }

    // 21.3.2 iterators:
    iterator begin()
    { return Storage::begin(); }

    const_iterator begin() const
    { return Storage::begin(); }

    iterator end()
    { return Storage::end(); }

    const_iterator end() const
    { return Storage::end(); }

    reverse_iterator rbegin()
    { return reverse_iterator(end()); }

    const_reverse_iterator rbegin() const
    { return const_reverse_iterator(end()); }

    reverse_iterator rend()
    { return reverse_iterator(begin()); }

    const_reverse_iterator rend() const
    { return const_reverse_iterator(begin()); }

    // 21.3.3 capacity:
    size_type size() const
    { return Storage::size(); }

    size_type length() const
    { return size(); }

    size_type max_size() const
    { return Storage::max_size(); }

    void resize(size_type n, value_type c)
    { Storage::resize(n, c); }

    void resize(size_type n)
    { resize(n, value_type()); }

    size_type capacity() const
    { return Storage::capacity(); }

    void reserve(size_type res_arg = 0)
    {
        Enforce(res_arg <= max_size(), static_cast<std::length_error*>(0), "");
        Storage::reserve(res_arg);
    }

    void clear()
    { resize(0); }

    bool empty() const
    { return size() == 0; }

    // 21.3.4 element access:
    const_reference operator[](size_type pos) const
    { return *(c_str() + pos); }

    reference operator[](size_type pos)
    { return *(begin() + pos); }

    const_reference at(size_type n) const
    {
        Enforce(n <= size(), static_cast<std::out_of_range*>(0), "");
        return (*this)[n];
    }

    reference at(size_type n)
    {
        Enforce(n < size(), static_cast<std::out_of_range*>(0), "");
        return (*this)[n];
    }

    // 21.3.5 modifiers:
    flex_string& operator+=(const flex_string& str)
    { return append(str); }

    flex_string& operator+=(const value_type* s)
    { return append(s); }

    flex_string& operator+=(const value_type c)
    {
        push_back(c);
        return *this;
    }

    flex_string& append(const flex_string& str)
    { return append(str.data(), str.length()); }

    flex_string& append(const flex_string& str, const size_type pos,
        size_type n)
    {
        const size_type sz = str.size();
        Enforce(pos <= sz, static_cast<std::out_of_range*>(0), "");
        Procust(n, sz - pos);
        return append(str.data() + pos, n);
    }

    flex_string& append(const value_type* s, const size_type n)
    {
        Invariant checker(*this);
        (void) checker;
        if (IsAliasedRange(s, s + n))
        {
            const size_type offset = s - &*begin();
            Storage::reserve(size() + n);
            s = &*begin() + offset;
        }
        Storage::append(s, s + n);
        return *this;
    }

    flex_string& append(const value_type* s)
    { return append(s, traits_type::length(s)); }

    flex_string& append(size_type n, value_type c)
    {
        resize(size() + n, c);
        return *this;
    }

    template<class InputIterator>
    flex_string& append(InputIterator first, InputIterator last)
    {
        insert(end(), first, last);
        return *this;
    }

    void push_back(const value_type c) // primitive
    {
        const size_type cap = capacity();
        if (size() == cap)
        {
            reserve(cap << 1u);
        }
        Storage::append(&c, &c + 1);
    }

    flex_string& assign(const flex_string& str)
    {
        if (&str == this) return *this;
        return assign(str.data(), str.size());
    }

    flex_string& assign(const flex_string& str, const size_type pos,
        size_type n)
    {
        const size_type sz = str.size();
        Enforce(pos <= sz, static_cast<std::out_of_range*>(0), "");
        Procust(n, sz - pos);
        return assign(str.data() + pos, str.data() + pos + n);
    }

    flex_string& assign(const value_type* s, const size_type n)
    {
        Invariant checker(*this);
        (void) checker;
        if (size() >= n)
        {
            std::copy(s, s + n, begin());
            resize(n);
        }
        else
        {
            const value_type *const s2 = s + size();
            std::copy(s, s2, begin());
            append(s2, n - size());
        }
        return *this;
    }

    flex_string& assign(const value_type* s)
    { 
#ifdef WIN_VC6
		///< VC6对模板支持不好，导致这里编译不过
		///< flex_string& assign(const value_type* s, const size_type n)将这里的代码拷贝进来
		const size_type n = traits_type::length(s);

		Invariant checker(*this);
		(void) checker;
		if (size() >= n)
		{
			std::copy(s, s + n, begin());
			resize(n);
		}
		else
		{
			const value_type *const s2 = s + size();
			std::copy(s, s2, begin());
			append(s2, n - size());
		}
		return *this;
#else
		return assign(s, traits_type::length(s)); 
#endif
	}

    template <class ItOrLength, class ItOrChar>
    flex_string& assign(ItOrLength first_or_n, ItOrChar last_or_c)
    { return replace(begin(), end(), first_or_n, last_or_c); }

    flex_string& insert(size_type pos1, const flex_string& str)
    { return insert(pos1, str.data(), str.size()); }

    flex_string& insert(size_type pos1, const flex_string& str,
        size_type pos2, size_type n)
    {
        Enforce(pos2 <= str.length(), static_cast<std::out_of_range*>(0), "");
        Procust(n, str.length() - pos2);
        return insert(pos1, str.data() + pos2, n);
    }

    flex_string& insert(size_type pos, const value_type* s, size_type n)
    {
        Enforce(pos <= length(), static_cast<std::out_of_range*>(0), "");
        insert(begin() + pos, s, s + n);
        return *this;
    }

    flex_string& insert(size_type pos, const value_type* s)
    { return insert(pos, s, traits_type::length(s)); }

    flex_string& insert(size_type pos, size_type n, value_type c)
    {
        Enforce(pos <= length(), static_cast<std::out_of_range*>(0), "");
        insert(begin() + pos, n, c);
        return *this;
    }

    iterator insert(const iterator p, const value_type c)
    {
        const size_type pos = p - begin();
        insert(p, 1, c);
        return begin() + pos;
    }

private:

    // Care must be taken when dereferencing some iterator types.
    //
    // Users can implement this function in their namespace if their storage uses a special iterator type,
    // the function will be found through ADL.
    template<class Iterator>
    const typename std::iterator_traits<Iterator>::value_type * DereferenceValidIterator(Iterator it) const
    {
      return &*it;
    }

    // Care must be taken when dereferencing a reverse iterators, hence this special case.
    // This isn't in the std namespace so as not to pollute it or create name clashes.
    template<typename Iterator>
    const typename std::iterator_traits<Iterator>::value_type * DereferenceValidIterator(
      std::reverse_iterator<Iterator
#ifdef NO_ITERATOR_TRAITS
	  , typename std::iterator_traits<Iterator>::value_type
#endif
	  > it) const
    {
      return &*--it;
    }

    // Determine if the range aliases the current string.
    //
    // This method cannot be const because calling begin/end on copy-on-write implementations must have side effects.
    // A const version wouldn't make the string unique through this call.
    template<class Iterator>
    bool IsAliasedRange(Iterator beginIterator, Iterator endIterator)
    {
      if(!empty() && beginIterator != endIterator)
      {
        typedef const typename std::iterator_traits<Iterator>::value_type * pointer;
        pointer myBegin(&*begin());
        pointer myEnd(&*begin() + size());
        pointer rangeBegin(DereferenceValidIterator(beginIterator));
        const std::less_equal<pointer> less_equal = std::less_equal<pointer>();
        if(less_equal(myBegin, rangeBegin) && less_equal(rangeBegin, myEnd))
          return true;
      }
      return false;
    }

    template <int i> class Selector {};

    flex_string& InsertImplDiscr(iterator p,
        size_type n, value_type c, Selector<1>)
    {
        Invariant checker(*this);
        (void) checker;
        assert(begin() <= p && p <= end());
        const size_type insertOffset(p - begin());
        const size_type originalSize(size());
        if(n < originalSize - insertOffset)
        {
            // The new characters fit within the original string.
            // The characters that are pushed back need to be moved because they're aliased.
            // The appended characters will all be overwritten by the move.
            append(n, value_type(0));
            value_type * start = (&*begin());
            flex_string_details::pod_move(start + insertOffset, start + originalSize, start + insertOffset + n);
            std::fill(start + insertOffset, start + insertOffset + n, c);
        }
        else
        {
            // The new characters exceed the original string.
            // The characters that are pushed back can simply be copied since they aren't aliased.
            // The appended characters will partly be overwritten by the copy.
            append(n, c);
            value_type * start = (&*begin());
            flex_string_details::pod_copy(start + insertOffset, start + originalSize, start + insertOffset + n);
            std::fill(start + insertOffset, start + originalSize, c);
        }
        return *this;
    }

    template<class InputIterator>
    flex_string& InsertImplDiscr(iterator i,
        InputIterator b, InputIterator e, Selector<0>)
    {
		typedef typename std::iterator_traits<InputIterator>::iterator_category iterator_category;	// 规避VC6编译错误
        InsertImpl(i, b, e, iterator_category());
        return *this;
    }

    template <class FwdIterator>
    void InsertImpl(iterator i,
        FwdIterator s1, FwdIterator s2, std::forward_iterator_tag)
    {
        if(s1 == s2)
        {
            // Insert an empty range.
            return;
        }

        if(IsAliasedRange(s1, s2))
        {
            // The source range is contained in the current string, copy it and recurse.
            const flex_string temporary(s1, s2);
			typedef typename std::iterator_traits<FwdIterator>::iterator_category iterator_category;	// 规避VC6编译错误
            InsertImpl(i, temporary.begin(), temporary.end(), iterator_category());
            return;
        }

        Invariant checker(*this);
        (void) checker;

        const size_type pos = i - begin();
#if defined(_MSC_VER) && (_MSC_VER < 1300)
        typedef typename std::iterator_traits<FwdIterator>::distance_type diff_t;
#else
        typedef typename std::iterator_traits<FwdIterator>::difference_type diff_t;
#endif
        const diff_t n2 = std::distance(s1, s2);
        assert(n2 >= 0);
        using namespace flex_string_details;
        assert(pos <= size());

        const diff_t maxn2 = capacity() - size();
        if (maxn2 < n2)
        {
          // Reallocate the string.
            assert(!IsAliasedRange(s1, s2));
            reserve(size() + n2);
            i = begin() + pos;
        }
        if (pos + n2 <= size())
        {
            //const iterator oldEnd = end();
            //Storage::append(oldEnd - n2, n2);
            //std::copy(i, oldEnd - n2, i + n2);
            const iterator tailBegin = end() - n2;
            Storage::append(tailBegin, tailBegin + n2);
            //std::copy(i, tailBegin, i + n2);
            std::copy(reverse_iterator(tailBegin), reverse_iterator(i),
                reverse_iterator(tailBegin + n2));
            std::copy(s1, s2, i);
        }
        else
        {
            FwdIterator t = s1;
            const size_type old_size = size();
            std::advance(t, old_size - pos);
            assert(std::distance(t, s2) >= 0);
            Storage::append(t, s2);
            Storage::append(data() + pos, data() + old_size);
            std::copy(s1, t, i);
        }
    }

    template <class InputIterator>
    void InsertImpl(iterator insertPosition,
        InputIterator inputBegin, InputIterator inputEnd, std::input_iterator_tag)
    {
        flex_string temporary(begin(), insertPosition);
        for (; inputBegin != inputEnd; ++inputBegin)
        {
            temporary.push_back(*inputBegin);
        }
        temporary.append(insertPosition, end());
        swap(temporary);
    }

public:
    template <class ItOrLength, class ItOrChar>
    void insert(iterator p, ItOrLength first_or_n, ItOrChar last_or_c)
    {
        Selector<std::numeric_limits<ItOrLength>::is_specialized> sel;
        InsertImplDiscr(p, first_or_n, last_or_c, sel);
    }

    flex_string& erase(size_type pos = 0, size_type n = npos)
    {
        Invariant checker(*this);
        (void) checker;
        Enforce(pos <= length(), static_cast<std::out_of_range*>(0), "");
        Procust(n, length() - pos);
        std::copy(begin() + pos + n, end(), begin() + pos);
        resize(length() - n);
        return *this;
    }

    iterator erase(iterator position)
    {
        const size_type pos(position - begin());
        erase(pos, 1);
        return begin() + pos;
    }

    iterator erase(iterator first, iterator last)
    {
        const size_type pos(first - begin());
        erase(pos, last - first);
        return begin() + pos;
    }

    // Replaces at most n1 chars of *this, starting with pos1 with the content of str
    flex_string& replace(size_type pos1, size_type n1, const flex_string& str)
    { return replace(pos1, n1, str.data(), str.size()); }

    // Replaces at most n1 chars of *this, starting with pos1,
    // with at most n2 chars of str starting with pos2
    flex_string& replace(size_type pos1, size_type n1, const flex_string& str,
        size_type pos2, size_type n2)
    {
        Enforce(pos2 <= str.length(), static_cast<std::out_of_range*>(0), "");
        return replace(pos1, n1, str.data() + pos2,
            Min(n2, str.size() - pos2));
    }

//
// Replaces at most n1 chars of *this, starting with pos,
// with at most n2 chars of str.
// str must have at least n2 chars.
//flex_string& replace(const size_type pos, size_type n1,
//const value_type* s1, const size_type n2)
//{
//Invariant checker(*this);
//(void) checker;
//Enforce(pos <= size(), (std::out_of_range*)0, "");
//Procust(n1, size() - pos);
//const iterator b = begin() + pos;
//return replace(b, b + n1, s1, s1 + n2);
//using namespace flex_string_details;
//const int delta = int(n2 - n1);
//const bool aliased = IsAliasedRange(s1, s1 + n2);
//
//// From here on we're dealing with an aliased replace
//if (delta <= 0)
//{
//// simple case, we're shrinking
//pod_move(s1, s1 + n2, &*begin() + pos);
//pod_move(&*begin() + pos + n1, &*end(), &*begin() + pos + n1 + delta);
//resize(size() + delta);
//return *this;
//}
//
//// From here on we deal with aliased growth
//if (capacity() < size() + delta)
//{
//// realloc the string
//const size_type offset = s1 - data();
//reserve(size() + delta);
//s1 = data() + offset;
//}
//
//const value_type* s2 = s1 + n2;
//value_type* d1 = &*begin() + pos;
//value_type* d2 = d1 + n1;
//
//const int tailLen = int(&*end() - d2);
//
//if (delta <= tailLen)
//{
//value_type* oldEnd = &*end();
//// simple case
//Storage::append(oldEnd - delta, delta);
//
//pod_move(d2, d2 + (tailLen - delta), d2 + delta);
//if (le(d2, s1))
//{
//pod_copy(s1 + delta, s2 + delta, d1);
//}
//else
//{
//// d2 > s1
//if (le(d2, s2))
//{
//pod_move(s1, d2, d1);
//pod_move(d2 + delta, s2 + delta, d1 + (d2 - s1));
//}
//else
//{
//pod_move(s1, s2, d1);
//}
//}
//}
//else
//{
//const size_type sz = delta - tailLen;
//Storage::append(s2 - sz, sz);
//Storage::append(d2, tailLen);
//pod_move(s1, s2 - (delta - tailLen), d1);
//}
//return *this;
//}

    // Replaces at most n1 chars of *this, starting with pos, with chars from s
    flex_string& replace(size_type pos, size_type n1, const value_type* s)
    { return replace(pos, n1, s, traits_type::length(s)); }

    // Replaces at most n1 chars of *this, starting with pos, with n2 occurences of c
    // consolidated with
    // Replaces at most n1 chars of *this, starting with pos,
    // with at most n2 chars of str.
    // str must have at least n2 chars.
    template <class StrOrLength, class NumOrChar>
    flex_string& replace(size_type pos, size_type n1,
        StrOrLength s_or_n2, NumOrChar n_or_c)
    {
        Invariant checker(*this);
        (void) checker;
        Enforce(pos <= size(), static_cast<std::out_of_range*>(0), "");
        Procust(n1, length() - pos);
        const iterator b = begin() + pos;
        return replace(b, b + n1, s_or_n2, n_or_c);
    }

    flex_string& replace(iterator i1, iterator i2, const flex_string& str)
    { return replace(i1, i2, str.data(), str.length()); }

    flex_string& replace(iterator i1, iterator i2, const value_type* s)
    { return replace(i1, i2, s, traits_type::length(s)); }

private:
    flex_string& ReplaceImplDiscr(iterator i1, iterator i2,
        const value_type* s, size_type n, Selector<2>)
    {
        assert(i1 <= i2);
        assert(begin() <= i1 && i1 <= end());
        assert(begin() <= i2 && i2 <= end());
        return replace(i1, i2, s, s + n);
    }

    flex_string& ReplaceImplDiscr(iterator i1, iterator i2,
        size_type n2, value_type c, Selector<1>)
    {
        const size_type n1 = i2 - i1;
        if (n1 > n2)
        {
            std::fill(i1, i1 + n2, c);
            erase(i1 + n2, i2);
        }
        else
        {
            std::fill(i1, i2, c);
            insert(i2, n2 - n1, c);
        }
        return *this;
    }

    template <class InputIterator>
    flex_string& ReplaceImplDiscr(iterator i1, iterator i2,
        InputIterator b, InputIterator e, Selector<0>)
    {
		typedef typename std::iterator_traits<InputIterator>::iterator_category iterator_category;	// 规避VC6编译错误
        ReplaceImpl(i1, i2, b, e, iterator_category());
        return *this;
    }

    template <class FwdIterator>
    void ReplaceImpl(iterator i1, iterator i2,
        FwdIterator s1, FwdIterator s2, std::forward_iterator_tag)
    {
        Invariant checker(*this);
        (void) checker;
#if defined(_MSC_VER) && (_MSC_VER < 1300)
        typedef typename std::iterator_traits<iterator>::distance_type diff_t;
#else
        typedef typename std::iterator_traits<iterator>::difference_type diff_t;
#endif
        const diff_t n1 = i2 - i1;
        assert(n1 >= 0);

#if defined(_MSC_VER) && (_MSC_VER < 1300)
        typedef typename std::iterator_traits<FwdIterator>::distance_type fwd_diff_t;
#else
        typedef typename std::iterator_traits<FwdIterator>::difference_type fwd_diff_t;
#endif
        const fwd_diff_t n2 = std::distance(s1, s2);
        assert(n2 >= 0);

        if (IsAliasedRange(s1, s2))
        {
            // Aliased replace, copy to new string.
            flex_string temporary;
            temporary.reserve(size() - n1 + n2);
            temporary.append(begin(), i1).append(s1, s2).append(i2, end());
            swap(temporary);
            return;
        }

        if (n1 > n2)
        {
            // Shrinks.
            std::copy(s1, s2, i1);
            erase(i1 + n2, i2);
        }
        else
        {
            // Grows.
            flex_string_details::copy_n(s1, n1, i1);
            std::advance(s1, n1);
            insert(i2, s1, s2);
        }
    }

    template <class InputIterator>
    void ReplaceImpl(iterator i1, iterator i2,
        InputIterator b, InputIterator e, std::input_iterator_tag)
    {
        flex_string temp(begin(), i1);
        temp.append(b, e).append(i2, end());
        swap(temp);
    }

public:
    template <class T1, class T2>
    flex_string& replace(iterator i1, iterator i2,
        T1 first_or_n_or_s, T2 last_or_c_or_n)
    {
        const bool
            num1 = std::numeric_limits<T1>::is_specialized,
            num2 = std::numeric_limits<T2>::is_specialized;
        return ReplaceImplDiscr(i1, i2, first_or_n_or_s, last_or_c_or_n,
            Selector<num1 ? (num2 ? 1 : -1) : (num2 ? 2 : 0)>());
    }

    size_type copy(value_type* s, size_type n, size_type pos = 0) const
    {
        Enforce(pos <= size(), static_cast<std::out_of_range*>(0), "");
        Procust(n, size() - pos);

        flex_string_details::pod_copy(
            data() + pos,
            data() + pos + n,
            s);
        return n;
    }

    void swap(flex_string& rhs)
    {
        Storage& srhs = rhs;
        this->Storage::swap(srhs);
    }

    // 21.3.6 string operations:
    const value_type* c_str() const
    { return Storage::c_str(); }

    const value_type* data() const
    { return Storage::data(); }

    allocator_type get_allocator() const
    { return Storage::get_allocator(); }

    size_type find(const flex_string& str, size_type pos = 0) const
    { return find(str.data(), pos, str.length()); }

    size_type find (const value_type* s, size_type pos, size_type n) const
    {
        const size_type count(size());
        if (n + pos > count)
            return npos;
        for (; pos + n <= count; ++pos)
        {
            if (traits_type::compare(data() + pos, s, n) == 0)
            {
                return pos;
            }
        }
        return npos;
    }

    size_type find (const value_type* s, size_type pos = 0) const
    { return find(s, pos, traits_type::length(s)); }

    size_type find (value_type c, size_type pos = 0) const
    { return find(&c, pos, 1); }

    size_type rfind(const flex_string& str, size_type pos = npos) const
    { return rfind(str.data(), pos, str.length()); }

    size_type rfind(const value_type* s, size_type pos, size_type n) const
    {
        if (n > length()) return npos;
        pos = Min(pos, length() - n);
        if (n == 0) return pos;

        const_iterator i(begin() + pos);
        for (; ; --i)
        {
            if (traits_type::eq(*i, *s)
                && traits_type::compare(&*i, s, n) == 0)
            {
                return i - begin();
            }
            if (i == begin()) break;
        }
        return npos;
    }

    size_type rfind(const value_type* s, size_type pos = npos) const
    { return rfind(s, pos, traits_type::length(s)); }

    size_type rfind(value_type c, size_type pos = npos) const
    { return rfind(&c, pos, 1); }

    size_type find_first_of(const flex_string& str, size_type pos = 0) const
    { return find_first_of(str.data(), pos, str.length()); }

    size_type find_first_of(const value_type* s,
        size_type pos, size_type n) const
    {
        if (pos > length() || n == 0) return npos;
        const_iterator i(begin() + pos),
            finish(end());
        for (; i != finish; ++i)
        {
            if (traits_type::find(s, n, *i) != 0)
            {
                return i - begin();
            }
        }
        return npos;
    }

    size_type find_first_of(const value_type* s, size_type pos = 0) const
    { return find_first_of(s, pos, traits_type::length(s)); }

    size_type find_first_of(value_type c, size_type pos = 0) const
    { return find_first_of(&c, pos, 1); }

    size_type find_last_of (const flex_string& str,
        size_type pos = npos) const
    { return find_last_of(str.data(), pos, str.length()); }

    size_type find_last_of (const value_type* s, size_type pos,
        size_type n) const
    {
        if (!empty() && n > 0)
        {
            pos = Min(pos, length() - 1);
            const_iterator i(begin() + pos);
            for (;; --i)
            {
                if (traits_type::find(s, n, *i) != 0)
                {
                    return i - begin();
                }
                if (i == begin()) break;
            }
        }
        return npos;
    }

    size_type find_last_of (const value_type* s,
        size_type pos = npos) const
    { return find_last_of(s, pos, traits_type::length(s)); }

    size_type find_last_of (value_type c, size_type pos = npos) const
    { return find_last_of(&c, pos, 1); }

    size_type find_first_not_of(const flex_string& str,
        size_type pos = 0) const
    { return find_first_not_of(str.data(), pos, str.size()); }

    size_type find_first_not_of(const value_type* s, size_type pos,
        size_type n) const
    {
        if (pos < length())
        {
            const_iterator
                i(begin() + pos),
                finish(end());
            for (; i != finish; ++i)
            {
                if (traits_type::find(s, n, *i) == 0)
                {
                    return i - begin();
                }
            }
        }
        return npos;
    }

    size_type find_first_not_of(const value_type* s,
        size_type pos = 0) const
    { return find_first_not_of(s, pos, traits_type::length(s)); }

    size_type find_first_not_of(value_type c, size_type pos = 0) const
    { return find_first_not_of(&c, pos, 1); }

    size_type find_last_not_of(const flex_string& str,
        size_type pos = npos) const
    { return find_last_not_of(str.data(), pos, str.length()); }

    size_type find_last_not_of(const value_type* s, size_type pos,
        size_type n) const
    {
        if (!this->empty())
        {
            pos = Min(pos, size() - 1);
            const_iterator i(begin() + pos);
            for (;; --i)
            {
                if (traits_type::find(s, n, *i) == 0)
                {
                    return i - begin();
                }
                if (i == begin()) break;
            }
        }
        return npos;
    }

    size_type find_last_not_of(const value_type* s,
        size_type pos = npos) const
    { return find_last_not_of(s, pos, traits_type::length(s)); }

    size_type find_last_not_of (value_type c, size_type pos = npos) const
    { return find_last_not_of(&c, pos, 1); }

    flex_string substr(size_type pos = 0, size_type n = npos) const
    {
        Enforce(pos <= size(), static_cast<std::out_of_range*>(0), "");
        return flex_string(data() + pos, Min(n, size() - pos));
    }

    int compare(const flex_string& str) const
    {
        // FIX due to Goncalo N M de Carvalho July 18, 2005
        return compare(0, size(), str);
    }

    int compare(size_type pos1, size_type n1,
        const flex_string& str) const
    { return compare(pos1, n1, str.data(), str.size()); }

    // FIX to compare: added the TC
    // (http://www.comeaucomputing.com/iso/lwg-defects.html number 5)
    // Thanks to Caleb Epstein for the fix

    int compare(size_type pos1, size_type n1,
        const value_type* s) const
    {
        return compare(pos1, n1, s, traits_type::length(s));
    }

    int compare(size_type pos1, size_type n1,
        const value_type* s, size_type n2) const
    {
        Enforce(pos1 <= size(), static_cast<std::out_of_range*>(0), "");
        Procust(n1, size() - pos1);
        // The line below fixed by Jean-Francois Bastien, 04-23-2007. Thanks!
        const int r = traits_type::compare(pos1 + data(), s, Min(n1, n2));
        return r != 0 ? r : n1 > n2 ? 1 : n1 < n2 ? -1 : 0;
    }

    int compare(size_type pos1, size_type n1,
        const flex_string& str,
        size_type pos2, size_type n2) const
    {
        Enforce(pos2 <= str.size(), static_cast<std::out_of_range*>(0), "");
        return compare(pos1, n1, str.data() + pos2, Min(n2, str.size() - pos2));
    }

    // Code from Jean-Francois Bastien (03/26/2007)
    int compare(const value_type* s) const
    {
      // Could forward to compare(0, size(), s, traits_type::length(s))
      // but that does two extra checks
      const size_type n1(size()), n2(traits_type::length(s));
      const int r = traits_type::compare(data(), s, Min(n1, n2));
      return r != 0 ? r : n1 > n2 ? 1 : n1 < n2 ? -1 : 0;
    }
};

template <typename E1, class T, class A, class S>
const typename flex_string<E1, T, A, S>::size_type
flex_string<E1, T, A, S>::npos = static_cast<typename flex_string<E1, T, A, S>::size_type>(-1);



////////////////////////////////////////////////////////////////////////////////

#define DAHUA_FLEX_STRING	flex_string

// non-member functions
template <typename E, class T, class A, class S>
DAHUA_FLEX_STRING<E, T, A, S> operator+(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{
    DAHUA_FLEX_STRING<E, T, A, S> result;
    result.reserve(lhs.size() + rhs.size());
    result.append(lhs).append(rhs);
    return result;
}

template <typename E, class T, class A, class S>
DAHUA_FLEX_STRING<E, T, A, S> operator+(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{
    DAHUA_FLEX_STRING<E, T, A, S> result;
    const typename DAHUA_FLEX_STRING<E, T, A, S>::size_type len =
        DAHUA_FLEX_STRING<E, T, A, S>::traits_type::length(lhs);
    result.reserve(len + rhs.size());
    result.append(lhs, len).append(rhs);
    return result;
}

template <typename E, class T, class A, class S>
DAHUA_FLEX_STRING<E, T, A, S> operator+(
    typename DAHUA_FLEX_STRING<E, T, A, S>::value_type lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{
    DAHUA_FLEX_STRING<E, T, A, S> result;
    result.reserve(1 + rhs.size());
    result.push_back(lhs);
    result.append(rhs);
    return result;
}

template <typename E, class T, class A, class S>
DAHUA_FLEX_STRING<E, T, A, S> operator+(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{
    typedef typename DAHUA_FLEX_STRING<E, T, A, S>::size_type size_type;
    typedef typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type traits_type;

    DAHUA_FLEX_STRING<E, T, A, S> result;
    const size_type len = traits_type::length(rhs);
    result.reserve(lhs.size() + len);
    result.append(lhs).append(rhs, len);
    return result;
}

template <typename E, class T, class A, class S>
DAHUA_FLEX_STRING<E, T, A, S> operator+(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    typename DAHUA_FLEX_STRING<E, T, A, S>::value_type rhs)
{
    DAHUA_FLEX_STRING<E, T, A, S> result;
    result.reserve(lhs.size() + 1);
    result.append(lhs);
    result.push_back(rhs);
    return result;
}

template <typename E, class T, class A, class S>
bool operator==(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return lhs.compare(rhs) == 0; }

template <typename E, class T, class A, class S>
bool operator==(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return rhs == lhs; }

template <typename E, class T, class A, class S>
bool operator==(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{ return lhs.compare(rhs) == 0; }

template <typename E, class T, class A, class S>
bool operator!=(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return !(lhs == rhs); }

template <typename E, class T, class A, class S>
bool operator!=(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return !(lhs == rhs); }

template <typename E, class T, class A, class S>
bool operator!=(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{ return !(lhs == rhs); }

template <typename E, class T, class A, class S>
bool operator<(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return lhs.compare(rhs) < 0; }

template <typename E, class T, class A, class S>
bool operator<(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{ return lhs.compare(rhs) < 0; }

template <typename E, class T, class A, class S>
bool operator<(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return rhs.compare(lhs) > 0; }

template <typename E, class T, class A, class S>
bool operator>(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return rhs < lhs; }

template <typename E, class T, class A, class S>
bool operator>(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{ return rhs < lhs; }

template <typename E, class T, class A, class S>
bool operator>(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return rhs < lhs; }

template <typename E, class T, class A, class S>
bool operator<=(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return !(rhs < lhs); }

template <typename E, class T, class A, class S>
bool operator<=(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{ return !(rhs < lhs); }

template <typename E, class T, class A, class S>
bool operator<=(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return !(rhs < lhs); }

template <typename E, class T, class A, class S>
bool operator>=(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return !(lhs < rhs); }

template <typename E, class T, class A, class S>
bool operator>=(const DAHUA_FLEX_STRING<E, T, A, S>& lhs,
    const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* rhs)
{ return !(lhs < rhs); }

template <typename E, class T, class A, class S>
bool operator>=(const typename DAHUA_FLEX_STRING<E, T, A, S>::value_type* lhs,
    const DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{ return !(lhs < rhs); }

template <typename E, class T, class A, class S>
inline void swap(DAHUA_FLEX_STRING<E, T, A, S>& lhs, DAHUA_FLEX_STRING<E, T, A, S>& rhs)
{
  // 21.3.7.8
  lhs.swap(rhs);
}

template <typename E, class T, class A, class S>
std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>&
operator>>(
    std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>& is,
    DAHUA_FLEX_STRING<E, T, A, S>& str);

template <typename E, class T, class A, class S>
std::basic_ostream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>&
operator<<(
    std::basic_ostream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>& os,
    const DAHUA_FLEX_STRING<E, T, A, S>& str)
{ return os << str.c_str(); }

template <typename E, class T, class A, class S>
std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>&
getline(
    std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
        typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>& is,
    DAHUA_FLEX_STRING<E, T, A, S>& str,
    typename DAHUA_FLEX_STRING<E, T, A, S>::value_type delim)
{
  size_t nread = 0;
  typename std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>::sentry sentry(is, true);
  if (sentry) {
    std::basic_streambuf<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
      typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>* buf = is.rdbuf();
    str.clear();

    while (nread < str.max_size()) {
      int c1 = buf->sbumpc();
      if (DAHUA_FLEX_STRING<E, T, A, S>::traits_type::eq_int_type(c1, DAHUA_FLEX_STRING<E, T, A, S>::traits_type::eof())) {
        is.setstate(std::ios_base::eofbit);
        break;
      }
      else {
        ++nread;
        typename DAHUA_FLEX_STRING<E, T, A, S>::value_type c = DAHUA_FLEX_STRING<E, T, A, S>::traits_type::to_char_type(c1);
        if (!DAHUA_FLEX_STRING<E, T, A, S>::traits_type::eq(c, delim))
          str.push_back(c);
        else
          break;              // Character is extracted but not appended.
      }
    }
  }
  if (nread == 0 || nread >= str.max_size())
    is.setstate(std::ios_base::failbit);

  return is;
}

template <typename E, class T, class A, class S>
std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
    typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>&
getline(
    std::basic_istream<typename DAHUA_FLEX_STRING<E, T, A, S>::value_type,
        typename DAHUA_FLEX_STRING<E, T, A, S>::traits_type>& is,
    DAHUA_FLEX_STRING<E, T, A, S>& str)
{
  return getline(is, str, is.widen('\n'));
}

} // namespace Infra
} // namespace Dahua

#undef DAHUA_FLEX_STRING


#endif // DAHUA_FLEX_STRING_SHELL_INC_
