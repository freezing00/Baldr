#ifndef DAHUA_DETAIL_ATOMIC_COUNT_HPP_INCLUDED
#define DAHUA_DETAIL_ATOMIC_COUNT_HPP_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

//
//  boost/detail/atomic_count.hpp - thread/SMP safe reference counter
//
//  Copyright (c) 2001, 2002 Peter Dimov and Multi Media Ltd.
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
//  typedef <implementation-defined> boost::detail::atomic_count;
//
//  atomic_count a(n);
//
//    (n is convertible to long)
//
//    Effects: Constructs an atomic_count with an initial value of n
//
//  a;
//
//    Returns: (long) the current value of a
//
//  ++a;
//
//    Effects: Atomically increments the value of a
//    Returns: nothing
//
//  --a;
//
//    Effects: Atomically decrements the value of a
//    Returns: (long) zero if the new value of a is zero,
//      unspecified non-zero value otherwise (usually the new value)
//
//    Important note: when --a returns zero, it must act as a
//      read memory barrier (RMB); i.e. the calling thread must
//      have a synchronized view of the memory
//
//    On Intel IA-32 (x86) memory is always synchronized, so this
//      is not a problem.
//
//    On many architectures the atomic instructions already act as
//      a memory barrier.
//
//    This property is necessary for proper reference counting, since
//      a thread can update the contents of a shared object, then
//      release its reference, and another thread may immediately
//      release the last reference causing object destruction.
//
//    The destructor needs to have a synchronized view of the
//      object to perform proper cleanup.
//
//    Original example by Alexander Terekhov:
//
//    Given:
//
//    - a mutable shared object OBJ;
//    - two threads THREAD1 and THREAD2 each holding
//      a private smart_ptr object pointing to that OBJ.
//
//    t1: THREAD1 updates OBJ (thread-safe via some synchronization)
//      and a few cycles later (after "unlock") destroys smart_ptr;
//
//    t2: THREAD2 destroys smart_ptr WITHOUT doing any synchronization
//      with respect to shared mutable object OBJ; OBJ destructors
//      are called driven by smart_ptr interface...
//


#if defined( __GNUC__ ) && ( defined( __i386__ ) || defined( __x86_64__ ) )

# include "atomic_count_gcc_x86.hpp"

#elif defined WIN64 || defined _WIN64 || defined(__WIN64__)

#  include "atomic_count_win64.hpp"

#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32__)

#  include "atomic_count_win32.hpp"

#elif defined( __GNUC__ ) && ( __GNUC__ * 100 + __GNUC_MINOR__ >= 401 )

#  include "atomic_count_sync.hpp"

#elif defined(__GLIBCPP__) || defined(__GLIBCXX__)

#  include "atomic_count_gcc.hpp"

#elif defined(__linux__)

#  include "atomic_count_pthreads.hpp"

#else

// Use #define DAHUA_DISABLE_THREADS to avoid the error
#error Unrecognized threading platform

#endif


#endif // #ifndef DAHUA_DETAIL_ATOMIC_COUNT_HPP_INCLUDED
