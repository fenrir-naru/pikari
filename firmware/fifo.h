#ifndef __FIFO_H__
#define __FIFO_H__

#include <stddef.h>

#define CONCAT3(str1, str2, str3) str1 ## str2 ## str3
#define CONCAT4(str1, str2, str3, str4) str1 ## str2 ## str3 ## str4

#ifndef FIFO_TYPE
#define FIFO_TYPE char
#endif
#define FIFO_T(type) CONCAT3(fifo_, type, _t)
#define FIFO_METHOD(type, name) CONCAT4(fifo_, type, _, name)

#ifndef FIFO_SIZE_T
#define FIFO_SIZE_T unsigned char
#endif

#ifndef FIFO_BUFFER_STORAGE
#define FIFO_BUFFER_STORAGE
#endif

typedef struct {
  FIFO_BUFFER_STORAGE FIFO_TYPE *buffer;
  FIFO_SIZE_T size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *follower;
} FIFO_T(FIFO_TYPE);

/* The mechanizm of the ring buffer.
 * 1) The "follower" can reach the "prius" , but not overtake it. 
 * 2) The "prius" can not reach the "follower" more than overtake it.
 */

FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, write) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *values, FIFO_SIZE_T size);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, put) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *value);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, read) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *buffer, FIFO_SIZE_T size);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, get) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *buffer);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, size) (FIFO_T(FIFO_TYPE) *fifo);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, margin) (FIFO_T(FIFO_TYPE) *fifo);
FIFO_T(FIFO_TYPE) * FIFO_METHOD(FIFO_TYPE,init) (FIFO_T(FIFO_TYPE) *fifo, FIFO_BUFFER_STORAGE FIFO_TYPE *buffer, FIFO_SIZE_T size);

#define FIFO_DIRECT_PUT(fifo, value, res) { \
  FIFO_BUFFER_STORAGE FIFO_TYPE *next = fifo.prius + 1; \
  res = 0; \
  if(next == (fifo.buffer + fifo.size)) next = fifo.buffer; \
  if(next != fifo.follower){ \
    *fifo.prius = value; \
    fifo.prius = next; \
    res = 1; \
  } \
}

#define FIFO_DIRECT_GET(fifo, buf, res) { \
  res = 0; \
  if(fifo.follower != fifo.prius){ \
    buf = *(fifo.follower++); \
    if(fifo.follower == fifo.buffer + fifo.size){ \
      fifo.follower = fifo.buffer; \
    } \
    res = 1; \
  } \
}

#endif /* __FIFO_H__ */
