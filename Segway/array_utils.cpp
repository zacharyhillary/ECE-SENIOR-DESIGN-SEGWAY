#include "array_utils.h"

void pushToArray(int* arrayOfInputs, int nextInput, int arrayLength) {
  int index = 0;
  while(index < arrayLength - 1) {
    arrayOfInputs[index] = arrayOfInputs[index+1];
    index++;
  }
  arrayOfInputs[index] = nextInput;
}

void pushToDoubleArray(double* arrayOfInputs, double nextInput, int arrayLength) {
  int index = 0;
  while(index < arrayLength - 1) { // 0, 1, 2, 3 -> 4
    arrayOfInputs[index] = arrayOfInputs[index+1];
    index++;
  }
  arrayOfInputs[index] = nextInput; // 4 reassigned down here
}

bool checkEachLessThan(int* array, int value, int arrayLength) {
    int index = 0;
    while (index < arrayLength) {
      if (array[index] >= value) {
        return false;
      }
      index++;
    }
    return true;
}

bool checkEachGreaterThan(int* array, int value, int arrayLength) {
    int index = 0;
    while (index < arrayLength) {
      if (array[index] <= value) {
        return false;
      }
      index++;
    }
    return true;
}

void clearIntArray(int* array, int arrayLength) {
  for(int i=0; i<arrayLength; i++) {
    array[i] = 0;
  }
}

double avgOfDoubleArray(double* arrayOfInputs, int arrayLength) {
  double sum = 0.0;
  for (int i=0; i<arrayLength; i++) {
    sum = sum + arrayOfInputs[i];
  }
  return sum / arrayLength;
}
