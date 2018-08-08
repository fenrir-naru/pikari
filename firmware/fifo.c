#include "fifo.h"
#include <string.h>

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef max
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

#define NULL_CHECK 0

/**
 * FIFO�����������܂��B
 * 
 * @param fifo
 * @param buffer
 * @param size 
 * @return ( FIFO_T(FIFO_TYPE) ) 
 */
FIFO_T(FIFO_TYPE) * FIFO_METHOD(FIFO_TYPE, init) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_BUFFER_STORAGE FIFO_TYPE * buffer, 
      FIFO_SIZE_T size){
  if(size > 0){
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->prius = fifo->buffer; 
    fifo->follower = fifo->buffer;
    return fifo;
  }else{return NULL;}
}

/**
 * FIFO�ɏ����o���܂�
 * 
 * @param fifo 
 * @param data 
 * @param size 
 * @return (int)
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, write) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *values, 
      FIFO_SIZE_T size){
  FIFO_SIZE_T _size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *prius_next;
#if NULL_CHECK
  if(values == NULL){return 0;}
#endif
  size = min(FIFO_METHOD(FIFO_TYPE, margin)(fifo), size);
  _size = fifo->buffer + fifo->size - fifo->prius;
  if(_size <= size){
    memcpy(fifo->prius, values, _size);
    prius_next = fifo->buffer;
    values += _size;
    _size = size - _size;
  }else{
    prius_next = fifo->prius;
    _size = size;
  }
  if(_size > 0){
    memcpy(prius_next, values, _size);
    prius_next += _size;
  }
  fifo->prius = prius_next;
  return size;
}

/**
 * FIFO�ɏ����o���܂�
 * 
 * @param fifo 
 * @param data 
 * @return (int)
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, put) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *value){
#if NULL_CHECK
  if(values == NULL){return 0;}
#endif
  {
    FIFO_BUFFER_STORAGE FIFO_TYPE *next = fifo->prius + 1;
    if(next == (fifo->buffer + fifo->size)) next = fifo->buffer;
    if(next != fifo->follower){
      *fifo->prius = *value;
      fifo->prius = next;
      return 1;
    }else{
      return 0;
    }
  }
}

/**
 * FIFO����ǂݍ��݂܂�
 * 
 * @param fifo 
 * @param buffer 
 * @param size 
 * @return (int) 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, read) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *buffer, 
      FIFO_SIZE_T size){
  FIFO_SIZE_T _size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *follower_next;
#if NULL_CHECK
  if(buffer == NULL){return 0;}
#endif
  size = min(FIFO_METHOD(FIFO_TYPE, size)(fifo), size);
  _size = fifo->buffer + fifo->size - fifo->follower;
  if(_size <= size){
    memcpy(buffer, fifo->follower, _size);
    follower_next = fifo->buffer;
    buffer += _size;
    _size = size - _size;
  }else{
    follower_next = fifo->follower;
    _size = size;
  }
  if(_size > 0){
    memcpy(buffer, follower_next, _size);
    follower_next += _size;
  }
  fifo->follower = follower_next;
  return size;
}

/**
 * FIFO����ǂݍ��݂܂�
 * 
 * @param fifo 
 * @param buffer 
 * @return (int) 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, get) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *buffer){
#if NULL_CHECK
  if(buffer == NULL){return 0;}
#endif
  if(fifo->follower != fifo->prius){
    *buffer = *(fifo->follower++);
    if(fifo->follower == fifo->buffer + fifo->size){
      fifo->follower = fifo->buffer;
    }
    return 1;
  }else{
    return 0;
  }
}

/**
 * FIFO�̏����ݍς݃f�[�^�傫����Ԃ��܂�
 * 
 * @param fifo 
 * @return int 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, size) (FIFO_T(FIFO_TYPE) *fifo){
  FIFO_BUFFER_STORAGE FIFO_TYPE *_prius = fifo->prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *_follower = fifo->follower;
  return (_prius >= _follower ?
           _prius - _follower:
           _prius + fifo->size - _follower);
}

/**
 * FIFO�̋󂫗̈�̑傫����Ԃ��܂�
 * 
 * @param fifo 
 * @return int 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, margin) (FIFO_T(FIFO_TYPE) *fifo){
  FIFO_BUFFER_STORAGE FIFO_TYPE *_prius = fifo->prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *_follower = fifo->follower;
  return (_follower <= _prius ?
           _follower + fifo->size - _prius - 1:
           _follower - _prius - 1);
}
