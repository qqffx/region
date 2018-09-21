#include <windows.h>
#include <stdint.h>
#include <stdio.h>


typedef struct {
  unsigned int delta;
  unsigned int theLast;
  unsigned int theNext;
  unsigned int dataAdress;
} debug_struct;


inline void printDebug(debug_struct *debugStruct) {
  
  fprintf(stderr, "delta: %d | index: %x | last: %d | next %d \n",
          debugStruct->delta,
          debugStruct->dataAdress,
          debugStruct->theLast,
          debugStruct->theNext);
}


int main(void) {
  
  char *filename = "orbita_data.bin";
  HANDLE fileHandle = CreateFile(filename, GENERIC_READ, 0, 0,
                                 OPEN_EXISTING, 0, 0);
  
#define NUMBER_OF_DATA_UNITS 1024
  unsigned long nNumberOfBytesToRead = NUMBER_OF_DATA_UNITS*2;
  unsigned long lpNumberOfBytesRead = 0;
  unsigned short int lpBuffer[NUMBER_OF_DATA_UNITS] = {0};
  
  debug_struct debugStruct;
  
  unsigned int Index = 0;
  unsigned int box = 0;
  unsigned int dataAdress = 0;
  unsigned int delta = 0;
  unsigned int fragCounter = 0;
  
  for(;;) {
    
    ReadFile(fileHandle, lpBuffer, nNumberOfBytesToRead, &lpNumberOfBytesRead, NULL);
    
    if (lpNumberOfBytesRead == 0) break;
    
    for(int i = 0; i < NUMBER_OF_DATA_UNITS; ++i) {
      if (lpBuffer[i] > box) {
        delta = lpBuffer[i] - box;
        box = lpBuffer[i];
      }
      else if(lpBuffer[i] < box) {
        delta = (65536 - box + lpBuffer[i]);
        box = lpBuffer[i];
      }
      else {
        fprintf(stderr, "exception \r\n");
      }
      
      if( abs(delta - 250) > 5 ) {
        debugStruct.delta = delta;
        debugStruct.dataAdress = dataAdress*2;
        debugStruct.theLast = lpBuffer[i - 1];
        debugStruct.theNext = lpBuffer[i];
        
        printDebug(&debugStruct);
        fragCounter++;
      }
      
      dataAdress++;
    }
  } // for(;;)
  
  fprintf(stderr, "error Counter %d \r\n", fragCounter);
        
  return(0);
}
  

