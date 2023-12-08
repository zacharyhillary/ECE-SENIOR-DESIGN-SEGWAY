#ifndef ARRAY_UTILS_H
#define ARRAY_UTILS_H

void pushToArray(int* arrayOfInputs, int nextInput, int arrayLength);
void pushToDoubleArray(double* arrayOfInputs, double nextInput, int arrayLength);
bool checkEachLessThan(int* array, int value, int arrayLength);
bool checkEachLessThan(double* array, double value, int arrayLength);
bool checkEachGreaterThan(int* array, int value, int arrayLength);
bool checkEachGreaterThan(double* array, double value, int arrayLength);
void clearIntArray(int* array, int arrayLength);
double avgOfIntArray(int* arrayOfInputs, int arrayLength);
double avgOfDoubleArray(double* arrayOfInputs, int arrayLength);

#endif /* ARRAY_UTILS_H */