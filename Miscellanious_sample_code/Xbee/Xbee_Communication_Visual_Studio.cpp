/***************************************
** Copyright ERTS Lab, IIT Bombay. 
** Licensed under Creative Commons Attribution 3.0 
** Author: Adithya G. (Lab intern)
**/
#include <stdio.h>
#include <time.h>
#include<conio.h>
HANDLE hPort;
DWORD iBytesWritten=0;
DWORD iBytesToRead =1;
void delay()
{
	int i,j;
	for(i=0; i<500;i++)
		for(j=0;j<500;j++);
}
void ClosePort(HANDLE hport)
{
    CloseHandle(hPort);
    return;
}

 HANDLE ConfigureSerialPort(LPCSTR  lpszPortName)
{
    HANDLE hComm = NULL;
    DWORD dwError;
    DCB PortDCB;
    COMMTIMEOUTS CommTimeouts;
    /// Open the serial port.
    hComm = CreateFile (lpszPortName, // Pointer to the name of the port
        GENERIC_READ | GENERIC_WRITE,
  //       Access (read-write) mode
        0,              // Share mode
        NULL,           // Pointer to the security attribute
        OPEN_EXISTING,  // How to open the serial port
        0,              // Port attributes
        NULL);          // Handle to port with attribute
     //to copy

    // Initialize the DCBlength member.
    PortDCB.DCBlength = sizeof (DCB);
     //Get the default port setting information.
    GetCommState (hComm, &PortDCB);
    // Change the DCB structure settings.
    PortDCB.BaudRate = 19200;              // Current baud
    PortDCB.fBinary = TRUE;               // Binary mode; no EOF check
    PortDCB.fParity = TRUE;               // Enable parity checking
    PortDCB.fOutxCtsFlow = FALSE;         // No CTS output flow control
    PortDCB.fOutxDsrFlow = FALSE;         // No DSR output flow control
    PortDCB.fDtrControl = DTR_CONTROL_ENABLE;
     //DTR flow control type
    PortDCB.fDsrSensitivity = FALSE;      // DSR sensitivity
    PortDCB.fTXContinueOnXoff = TRUE;     // XOFF continues Tx
    PortDCB.fOutX = FALSE;                // No XON/XOFF out flow control
    PortDCB.fInX = FALSE;                 // No XON/XOFF in flow control
    PortDCB.fErrorChar = FALSE;           // Disable error replacement
    PortDCB.fNull = FALSE;                // Disable null stripping
    PortDCB.fRtsControl = RTS_CONTROL_ENABLE;
     //RTS flow control
    PortDCB.fAbortOnError = FALSE;        // Do not abort reads/writes on
     //error
    PortDCB.ByteSize = 8;                 // Number of bits/byte, 4-8
    PortDCB.Parity = NOPARITY;            // 0-4=no,odd,even,mark,space
    PortDCB.StopBits = ONESTOPBIT;        // 0,1,2 = 1, 1.5, 2

     //Configure the port according to the specifications of the DCB
     //structure.
    if (!SetCommState (hComm, &PortDCB))
    {
        printf("Could not configure serial port\n");
        return NULL;
    }
    //Retrieve the time-out parameters for all read and write operations
     //on the port.
    GetCommTimeouts (hComm, &CommTimeouts);
     //Change the COMMTIMEOUTS structure settings.
    CommTimeouts.ReadIntervalTimeout = MAXDWORD;
    CommTimeouts.ReadTotalTimeoutMultiplier = 0;
    CommTimeouts.ReadTotalTimeoutConstant = 0;
    CommTimeouts.WriteTotalTimeoutMultiplier = 0;
    CommTimeouts.WriteTotalTimeoutConstant = 0;
    if (!SetCommTimeouts (hComm, &CommTimeouts))
    {
        printf("Could not set timeouts\n");
        return NULL;
    }
    return hComm;
}
 
 BOOL WriteByte(BYTE bybyte)
{
	DWORD *pointerToiBytesToRead;
	pointerToiBytesToRead = &iBytesWritten;
	
    if(WriteFile(hPort,(LPCVOID) 
        &bybyte,iBytesToRead,pointerToiBytesToRead,NULL)==0)
        return FALSE; 
    else return TRUE;
}

 BOOL ReadByte(BYTE  *resp)
{
    BOOL bReturn = TRUE;
    BYTE rx;
    DWORD dwBytesTransferred=0;

    if (ReadFile (hPort, &rx, 1, &dwBytesTransferred, 0)> 0)
    {
        if (dwBytesTransferred == 1)
        {
            *resp=rx;
            bReturn  = TRUE;
        }
        else bReturn = FALSE;
    }
    else    bReturn = FALSE;
    return bReturn;
}
	
int main()
{	
	int i=5;
	unsigned char receivedByte[4];
	hPort = ConfigureSerialPort("COM5");
	
	
	if(hPort == NULL)
	{
		printf("Com port configuration failed\n");
		getchar();
		cvWaitKey(0);
		return -1;
		
	}
	else
		printf("Com port configuration success\n");
	while(i==0)
	{
		
		WriteByte(0x0f);
		ReadByte(&receivedByte[0]);
		ReadByte(&receivedByte[1]);
		ReadByte(&receivedByte[2]);
		printf("%d%d%d\n",&receivedByte[0],&receivedByte[1],&receivedByte[2]);
	//Values taken here are ASCII values of Arrows in keyboard 	
		char c1=getch();
		printf("%d",c1);

		if(c1==-3272)
		{
		WriteByte(0x38);
		}
		else if(c1==-3275)
		{
		WriteByte(0x32);
}
else if(c1==-3277)
		{
		WriteByte(0x34);
		}
		else if(c1==-3280)
		{
		WriteByte(0x36);
		}
		else if(c1==13)//carriage return
		{
		WriteByte(0x00);
		}
		else
		c1=0;	
	}
WriteByte(Stop);
	WriteByte(Stop);
	ClosePort(hPort);
	i--;
}



