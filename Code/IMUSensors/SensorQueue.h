#ifndef SENSOR_QUEUE_H
#define SENSOR_QUEUE_H

#include <cppQueue.h>

class SensorQueue {
public:
  SensorQueue();
  
  // Push a value onto the queue
  void push(float value);
  
  // Get value at specific index
  float at(int index);
  
  // Set value at specific index
  void set(int index, float value);
  
  // Calculate weighted average of values in queue
  float weightedAverage(const float* weights, int sampleCount);
  
  // Get the current number of items in the queue
  int getCount() const;
  
  // Check if queue is full
  bool isFull() const;
  
  // Peek at value by index
  bool peekIdx(float* val, int idx);
  
  // Pop a value from the queue
  bool pop(float* val);

private:
  cppQueue queue;
  static const int QUEUE_SIZE = 10;
};

#endif // SENSOR_QUEUE_H