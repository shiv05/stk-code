// File added to FFLL so that it can be built on other platforms that WIN


// define wcsicmp for MAC OS X
#if defined ( __APPLE__ )
#include <fstream>
#define stricmp strcasecmp

inline int wcsicmp(const wchar_t* s1, const wchar_t* s2)
{
    char str1[128], str2[128];
    sprintf(str1, "%S", s1);
    sprintf(str2, "%S", s2);
    return stricmp(str1, str2);
}
#endif