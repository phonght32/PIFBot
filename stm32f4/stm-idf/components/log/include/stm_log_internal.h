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

#ifndef _LOG_INTERNAL_H_
#define _LOG_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif


/*
 * @brief   Output log buffer hex.
 * @param   tag Tag description.
 * @param   buffer Buffer.
 * @param   buff_len length.
 * @param   log_level Log output level.
 * @return  None.
 */
void stm_log_buffer_hex_internal(const char *tag,
                                 const char *buffer,
                                 uint16_t buff_len,
                                 stm_log_level_t log_level);

/*
 * @brief   Output log buffer char.
 * @param   tag Tag description.
 * @param   buffer Buffer.
 * @param   buff_len length.
 * @param   log_level Log output level.
 * @return  None.
 */
void stm_log_buffer_char_internal(const char *tag,
                                  const char *buffer,
                                  uint16_t buff_len,
                                  stm_log_level_t log_level);

#ifdef __cplusplus
}
#endif

#endif /* _LOG_INTERNAL_H_ */
