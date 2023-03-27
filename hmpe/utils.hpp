//
// Created by User on 14/03/2023.
//

#ifndef HMPE_UTILS_HPP
#define HMPE_UTILS_HPP

#include <memory>

#define HMPE_START namespace HMPE {
#define HMPE_END }

#ifdef __WIN32__
    #ifdef HMPE_DLL_EXPORT
        #define HMPE_API __declspec(dllexport)
    #elif HMPE_DLL_IMPORT
        #define HMPE_API __declspec(dllimport)
    #else
        #define HMPE_API
    #endif
#elif __linux__
    #error "Linux is not supported"
#elif __APPLE__
    #error "MacOS is not supported"
#else
    #error "Platform not supported"
#endif

HMPE_START

template <typename Type>
using Unique = std::unique_ptr<Type>;

template <typename Type>
using Shared = std::shared_ptr<Type>;

template <typename Type>
using Weak = std::weak_ptr<Type>;

class Body;

typedef Body* BodyRef;

HMPE_END
#endif //HMPE_UTILS_HPP
