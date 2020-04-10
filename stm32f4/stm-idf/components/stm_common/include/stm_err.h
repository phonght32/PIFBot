// MIT License

// Copyright (c) 2020 thanhphong98 & thuanpham98

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t stm_err_t;

#define STM_OK                      0
#define STM_FAIL                    -1

#define STM_ERR_NO_MEM              0x101
#define STM_ERR_INVALID_ARG         0x102
#define STM_ERR_INVALID_STATE       0x103
#define STM_ERR_INVALID_SIZE        0x104
#define STM_ERR_NOT_FOUND           0x105
#define STM_ERR_NOT_SUPPORTED       0x106
#define STM_ERR_TIMEOUT             0x107
#define STM_ERR_INVALID_RESPONSE    0x108
#define STM_ERR_INVALID_CRC         0x109
#define STM_ERR_INVALID_VERSION     0x10A


#ifdef __cplusplus
}
#endif