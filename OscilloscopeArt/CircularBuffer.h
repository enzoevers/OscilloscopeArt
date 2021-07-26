#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

template<class T>
class circularBuffer
{
  public:
    circularBuffer(const uint16_t bufferSize)
    : _bufferSize(bufferSize)
    {
      _buffer = new T[_bufferSize];
    }

    bool enqueue(T data) volatile
    { 
      if (isFull())
      {
        return false;
      }

      _buffer[_write] = data;
      _write++;
      if (_write >= _bufferSize)
      {
        _write = 0;
      }

      _curBufferSize++;
      return true;
    }

    bool dequeue(T& returnValue) volatile
    {
      if (isEmpty())
      {
        return false;
      }

      returnValue = _buffer[_read];
      _read++;
      if (_read >= _bufferSize)
      {
        _read = 0;
      }

      _curBufferSize--;
      return true;
    }

    bool isEmpty() volatile 
    {
      return _curBufferSize == 0;
    }

    bool isFull() volatile
    {
      return _curBufferSize == _bufferSize;
    }

    uint16_t getSize() volatile
    {
      return _curBufferSize;
    }
    uint16_t getMaxSize() volatile
    {
      return _bufferSize;
    }

  private:
    volatile uint16_t _write = 0;
    volatile uint16_t _read = 0;
    volatile uint16_t _curBufferSize = 0;

    volatile const uint16_t _bufferSize = 0;
    T* _buffer = nullptr;
};

#endif /* CIRCULAR_BUFFER_H */
