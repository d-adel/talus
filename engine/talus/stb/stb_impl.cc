#if defined(_MSC_VER)
  #pragma warning(push, 0)
#elif defined(__clang__) || defined(__GNUC__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wall"
  #pragma GCC diagnostic ignored "-Wextra"
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#if defined(_MSC_VER)
  #pragma warning(pop)
#elif defined(__clang__) || defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif
