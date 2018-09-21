#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

int64_t GlobalCounterFrequency;

inline float Win32GetSecondsElapsed(LARGE_INTEGER Start, LARGE_INTEGER End)
{
  float Result = (float)(End.QuadPart - Start.QuadPart) /
    (float)GlobalCounterFrequency;
  return(Result);
}

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

LARGE_INTEGER startCounter, endCounter;
LARGE_INTEGER fullCounter, fullCounterEnd;
float time = 0;
float fullTime = 0;

int main()
{
  // Define the five bytes to send ("hello")
  char* bytes_to_send = "send_data";
  
  // Declare variables and structures
  HANDLE hSerial;
  DCB dcbSerialParams = {0};
  COMMTIMEOUTS timeouts = {0};
  
  // Open the highest available serial port number
  fprintf(stderr, "Opening serial port...");
  hSerial = CreateFile(
    "\\\\.\\COM6", GENERIC_READ|GENERIC_WRITE, 0, NULL,
    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
  if (hSerial == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "Error\n");
    return 1;
  }
  else fprintf(stderr, "OK\n");
  
  //SetupComm(hSerial, 1200, 1200);
  
  // Set device parameters (38400 baud, 1 start bit,
  // 1 stop bit, no parity)
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (GetCommState(hSerial, &dcbSerialParams) == 0)
  {
    fprintf(stderr, "Error getting device state\n");
    CloseHandle(hSerial);
    return 1;
  }
  /*
  dcbSerialParams.BaudRate = CBR_38400;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  if(SetCommState(hSerial, &dcbSerialParams) == 0)
  {
    fprintf(stderr, "Error setting device parameters\n");
    DWORD lastError = GetLastError();
    CloseHandle(hSerial);
    return 1;
  }
  */
  // Set COM port timeout settings
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 50;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  timeouts.WriteTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  if(SetCommTimeouts(hSerial, &timeouts) == 0)
  {
    fprintf(stderr, "Error setting timeouts\n");
    CloseHandle(hSerial);
    return 1;
  }
  
  // Send specified text (remaining command line arguments)
#if 1
  DWORD bytes_written, total_bytes_written = 0;
  fprintf(stderr, "Sending bytes...");
  if(!WriteFile(hSerial, bytes_to_send, strlen(bytes_to_send), &bytes_written, NULL))
  {
    fprintf(stderr, "Error\n");
    CloseHandle(hSerial);
    return 1;
  }   
  fprintf(stderr, "%d bytes written\n", bytes_written);
  fprintf(stderr, "----------------\n", bytes_written);
#endif
  char *Filename = "orbita_data.bin";
  
  // this should always create new file data be written into
  HANDLE FileHandle = CreateFile(Filename, GENERIC_WRITE, 0, 0,
                                 CREATE_ALWAYS, 0, 0);
  if(FileHandle != INVALID_HANDLE_VALUE) {
    // NOTE(Egor): succeed
  }
  else {
    // NOTE(Egor); fail
  }
  
  debug_struct debugStruct = {0};
  
  
  LARGE_INTEGER CounterFrequencyResult;
  QueryPerformanceFrequency(&CounterFrequencyResult);
  GlobalCounterFrequency = CounterFrequencyResult.QuadPart;
#define NUMBER_OF_DATA_UNITS 15360
  unsigned short int lpBuffer[NUMBER_OF_DATA_UNITS] = {0};
  unsigned long nNumberOfBytesToRead = NUMBER_OF_DATA_UNITS*2;
  unsigned long lpNumberOfBytesRead;  
  
  unsigned int Index = 0;
  unsigned int box = 0;
  unsigned int dataAdress = 0;
  unsigned int delta = 0;
  
  QueryPerformanceCounter(&fullCounter);
  
  for(;;) {
    
    QueryPerformanceCounter(&startCounter);  
    ReadFile(
      hSerial,
      lpBuffer,
      nNumberOfBytesToRead,
      &lpNumberOfBytesRead,
      NULL
      );
    
    if(!strcmp((char *)lpBuffer, "end")) {
      CloseHandle(FileHandle);
      //char *copyString = "copy";
      //WriteFile(hSerial, copyString , strlen(copyString), &bytes_written, NULL);
      break;
    }
    else if(lpNumberOfBytesRead > 0) {
      
      //calculate Delta 
#if 0
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
        }
        
        dataAdress++;
      }
#endif
      
      
      
      // NOTE(Egor): succeed
      QueryPerformanceCounter(&endCounter);
      time += Win32GetSecondsElapsed(startCounter, endCounter);
      
      DWORD BytesWritten;
      // write data to file
      WriteFile(FileHandle, lpBuffer, lpNumberOfBytesRead, &BytesWritten, 0);
    }
      
  }// for(;;)

  
  QueryPerformanceCounter(&fullCounterEnd);
  fullTime = Win32GetSecondsElapsed(fullCounter, fullCounterEnd);
  
  
  
  char DebugBuffer[256];
  
  fprintf(stderr, "time: %f \n", time);
  
  fprintf(stderr, "full time: %f \n", fullTime);
  

          
  
  // Close serial port
  fprintf(stderr, "Closing serial port...");
  if (CloseHandle(hSerial) == 0)
  {
    fprintf(stderr, "Error\n");
    return 1;
  }
  fprintf(stderr, "OK\n");
  
  // exit normally
  return 0;
}