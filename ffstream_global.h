//
// Created by <Zifeng Deng>binape@126.com on 2024/7/17
//

#pragma once

#  ifdef WIN32
#    define FF_DECL_EXPORT     __declspec(dllexport)
#    define FF_DECL_IMPORT     __declspec(dllimport)
#  elif defined(FF_VISIBILITY_AVAILABLE)
#    define FF_DECL_EXPORT     __attribute__((visibility("default")))
#    define FF_DECL_IMPORT     __attribute__((visibility("default")))
#    define FF_DECL_HIDDEN     __attribute__((visibility("hidden")))
#  endif

#if defined(FFDECODER_LIBRARY)
#  define FFDECODER_EXPORT FF_DECL_EXPORT
#else
#  define FFDECODER_EXPORT FF_DECL_IMPORT
#endif
