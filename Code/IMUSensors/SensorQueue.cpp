#include "SensorQueue.h"

SensorQueue::SensorQueue()
  : queue(sizeof(float), QUEUE_SIZE, FIFO)  // Use constant for queue size
{
  float zero = 0.0f;
  for (int i = 0; i < QUEUE_SIZE; i++) {
    queue.push(&zero); // pre-fill with zeros
  }
}

void SensorQueue::push(float value) {
  float val = value; // Create a local copy to get its address
  queue.push(&val);
}

float SensorQueue::at(int index) {
  float val;
  if (queue.peekIdx(&val, index)) {
    return val;
  }
  return 0.0f; // Default value if index is invalid
}

void SensorQueue::set(int index, float value) {
  // This is a simplification - for a real implementation,
  // you would need to access the internal array of cppQueue
  // or rebuild the queue with the new value
  // For now, we'll implement a basic version that works only for
  // the specific use case in FilterManager
  
  // Create a temporary array to store all values
  float* values = new float[QUEUE_SIZE];
  
  // Extract all values
  for (int i = 0; i < QUEUE_SIZE; i++) {
    float val = 0.0f;
    queue.peekIdx(&val, i);
    values[i] = val;
  }
  
  // Update the value at the specified index
  if (index >= 0 && index < QUEUE_SIZE) {
    values[index] = value;
  }
  
  // Clear the queue and refill with the updated values
  while (!queue.isEmpty()) {
    float temp;
    queue.pop(&temp);
  }
  
  for (int i = 0; i < QUEUE_SIZE; i++) {
    queue.push(&values[i]);
  }
  
  delete[] values;
}

float SensorQueue::weightedAverage(const float* weights, int sampleCount) {
  float sum = 0;
  float weightSum = 0;
  float val;

  for (int i = 0; i < sampleCount && i < QUEUE_SIZE; i++) {
    queue.peekIdx(&val, i);
    sum += val * weights[i];
    weightSum += weights[i];
  }

  return (weightSum > 0) ? (sum / weightSum) : 0.0f;
}

int SensorQueue::getCount() const {
  // Cast away const to work with cppQueue which doesn't have const methods
  return const_cast<cppQueue&>(queue).getCount();
}

bool SensorQueue::isFull() const {
  // Cast away const to work with cppQueue which doesn't have const methods
  return const_cast<cppQueue&>(queue).isFull();
}

bool SensorQueue::peekIdx(float* val, int idx) {
  return queue.peekIdx(val, idx);
}

bool SensorQueue::pop(float* val) {
  return queue.pop(val);
}