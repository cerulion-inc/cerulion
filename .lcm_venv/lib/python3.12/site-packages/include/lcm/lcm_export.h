
#ifndef LCM_EXPORT_H
#define LCM_EXPORT_H

#ifdef LCM_STATIC
#  define LCM_EXPORT
#  define LCM_NO_EXPORT
#else
#  ifndef LCM_EXPORT
#    ifdef lcm_EXPORTS
        /* We are building this library */
#      define LCM_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define LCM_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef LCM_NO_EXPORT
#    define LCM_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef LCM_DEPRECATED
#  define LCM_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef LCM_DEPRECATED_EXPORT
#  define LCM_DEPRECATED_EXPORT LCM_EXPORT LCM_DEPRECATED
#endif

#ifndef LCM_DEPRECATED_NO_EXPORT
#  define LCM_DEPRECATED_NO_EXPORT LCM_NO_EXPORT LCM_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef LCM_NO_DEPRECATED
#    define LCM_NO_DEPRECATED
#  endif
#endif

#endif /* LCM_EXPORT_H */
