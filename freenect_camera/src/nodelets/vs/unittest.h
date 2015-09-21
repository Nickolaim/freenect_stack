#pragma once

#pragma warning (disable: 4505) // See https://support.microsoft.com/en-us/kb/947783 why disable/re-enable for this warning is not working properly.
#include "CppUnitTest.h"

namespace Microsoft{
  namespace VisualStudio {
    namespace CppUnitTestFramework
    {
      template<> static std::wstring ToString<unsigned short>(const unsigned short& t)         { RETURN_WIDE_STRING(t); }
    }
  }
}
