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

#ifndef DAHUA_FLEX_STRING_DETAILS_INC_
#define DAHUA_FLEX_STRING_DETAILS_INC_

// revision 754


#include <cstring>		// for memmove memcpy
#include <iterator>
#include <utility>		// for vc6 iterator_traits

namespace Dahua {
namespace Infra {


namespace flex_string_details
{
    template <class InIt, class OutIt>
#if defined(_MSC_VER) && (_MSC_VER < 1300)
	OutIt copy_n(InIt b, typename std::iterator_traits<InIt>::distance_type n, OutIt d)
#else
    OutIt copy_n(InIt b, typename std::iterator_traits<InIt>::difference_type n, OutIt d)
#endif
    {
        for (; n != 0; --n, ++b, ++d)
        {
            *d = *b;
        }
        return d;
    }

    template <class Pod, class T>
    inline void pod_fill(Pod* b, Pod* e, T c)
    {
        switch ((e - b) & 7)
        {
        case 0:
            while (b != e)
            {
                *b = c; ++b;
        case 7: *b = c; ++b;
        case 6: *b = c; ++b;
        case 5: *b = c; ++b;
        case 4: *b = c; ++b;
        case 3: *b = c; ++b;
        case 2: *b = c; ++b;
        case 1: *b = c; ++b;
            }
        }
    }

    template <class Pod>
    inline void pod_move(const Pod* b, const Pod* e, Pod* d)
    {
        using namespace std;
        memmove(d, b, (e - b) * sizeof(*b));
    }

    template <class Pod>
    inline Pod* pod_copy(const Pod* b, const Pod* e, Pod* d)
    {
        const size_t s = e - b;
        using namespace std;
        memcpy(d, b, s * sizeof(*b));
        return d + s;
    }

    template <typename T> struct get_unsigned
    {
        typedef T result;
    };

    template <> struct get_unsigned<char>
    {
        typedef unsigned char result;
    };

    template <> struct get_unsigned<signed char>
    {
        typedef unsigned char result;
    };

    template <> struct get_unsigned<short int>
    {
        typedef unsigned short int result;
    };

    template <> struct get_unsigned<int>
    {
        typedef unsigned int result;
    };

    template <> struct get_unsigned<long int>
    {
        typedef unsigned long int result;
    };

    enum Shallow {};
}

} // namespace Infra
} // namespace Dahua

#endif // DAHUA_FLEX_STRING_DETAILS_INC_
