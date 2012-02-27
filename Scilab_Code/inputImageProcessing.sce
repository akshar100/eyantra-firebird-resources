//********************************************************************************************
//*** slLoad : Used to load the siseli.dll file to use the serial port for zigbee comunication
//********************************************************************************************
function slLoad()
    slFuncs(1,1)= link("siseli.dll","version","c");
    slFuncs(1,2)= link("siseli.dll","mount","c");
    slFuncs(1,3)= link("siseli.dll","umount","c");
    slFuncs(1,4)= link("siseli.dll","check","c");
    slFuncs(1,5)= link("siseli.dll","config","c");
    slFuncs(1,6)= link("siseli.dll","open","c");
    slFuncs(1,7)= link("siseli.dll","close","c");
    slFuncs(1,8)= link("siseli.dll","sendb","c");
    slFuncs(1,9)= link("siseli.dll","senda","c");
    slFuncs(1,10)=link("siseli.dll","count","c");
    slFuncs(1,11)=link("siseli.dll","recvb","c");
    slFuncs=resume(slFuncs)
endfunction
//*****************************************************************************
//*** slVersion 
//*****************************************************************************
function res=slVersion()
    res=fort("version","out",[1,1],1,"i")/10
endfunction    


//*****************************************************************************
//*** slUnload
//*****************************************************************************
function slUnload()
    for i=1:size(slFuncs,2)
        ulink(slFuncs(i))
    end
    clear slFuncs
endfunction    


//*****************************************************************************
//*** slMount : Mount a port
//*****************************************************************************
function res=slMount(nHandle)
    res=fort("mount",nHandle,1,"i","out",[1,1],2,"i")
endfunction


//*****************************************************************************
//*** slUMount: Un mount a port
//*****************************************************************************
function res=slUMount(nHandle)
    res=fort("umount",nHandle,1,"i","out",[1,1],2,"i")
endfunction


//*****************************************************************************
//*** slCheck: check whether the serial port is workingor not
//*****************************************************************************
function res=slCheck(nHandle, nPort)
    res=fort("check",nHandle,1,"i",nPort,2,"i","out",[1,1],3,"i")
endfunction


//*****************************************************************************
//*** slConfig: configure the port.
//*****************************************************************************
function res=slConfig(nHandle, nBaud, nBits, nPar, nStop)
    res=fort("config",nHandle,1,"i",nBaud,2,"i",nBits,3,"i",nPar,4,"i",nStop,5,"i","out",[1,1],6,"i")
endfunction


//*****************************************************************************
//*** slOpen: open the port
//*****************************************************************************
function res=slOpen(nHandle, nPort)
    res=fort("open",nHandle,1,"i",nPort,2,"i","out",[1,1],3,"i")
endfunction


//*****************************************************************************
//*** slClose : close the port
//*****************************************************************************
function res=slClose(nHandle)
    res=fort("close",nHandle,1,"i","out",[1,1],2,"i")
endfunction


//*****************************************************************************
//*** slSendByte : sent byte through port
//*****************************************************************************
function res=slSendByte(nHandle, nByte)
    res=fort("sendb",nHandle,1,"i",nByte,2,"i","out",[1,1],3,"i")
endfunction


//*****************************************************************************
//*** slSendArray: send array through port
//*****************************************************************************
function res=slSendArray(nHandle, nByte, nLength)
    res=fort("senda",nHandle,1,"i",nByte,2,"i",nLength,3,"i","out",[1,1],4,"i")
endfunction


//*****************************************************************************
//*** slCount 
//*****************************************************************************
function res=slCount(nHandle)
    res=fort("count",nHandle,1,"i","out",[1,1],2,"i")
endfunction


//*****************************************************************************
//*** slCount
//*****************************************************************************
function res=slReadByte(nHandle, nBlock)
    res=fort("recvb",nHandle,1,"i",nBlock,2,"i","out",[1,1],3,"i")
endfunction




port_no = 9;	// This is the port no where the Zigbee is conneted ( COM 9 ) . You may have to change the value according to the COM port you are using.

inputImageBuffer = imread('C:\Documents and Settings\ERTS 12\Desktop\KSK\scilab_code\test2.bmp')  // This is the location of the Input image.

[rows,cols] = size(inputImageBuffer);
color1=zeros(1,rows*cols);
count=zeros(1,rows*cols);
inputImageBuffer2=zeros(rows,cols);
x=5;		// Color value
y=-30;		// row position
i=0;
r=1;
c=1;
j=1;
k=1;
// Setting the color value into the image array
for r=1:rows
	for c=1:cols
		if inputImageBuffer(r,c)==255 then,
			inputImageBuffer(r,c)=1;
		end
	end,
end,
// Inverting the even rows for runlength encoding.
for r=1:rows
	for c=1:cols
		if modulo(r,2) == 1 then,
			inputImageBuffer2(r,c)=inputImageBuffer(r,c);
		else
			inputImageBuffer2(r,c)=inputImageBuffer(r,cols-c+1);
		end
	end,
end,
inputImageBuffer=inputImageBuffer2;
//disp(inputImageBuffer);

//Run-length encoding
for r=1:rows
	for c=1:cols
		if inputImageBuffer(r,c)==x & y==r then,
			count(i)=count(i)+1;
		else
			color1(j)=inputImageBuffer(r,c);
			j=j+1;
			y=r;
			i=i+1;
			x=inputImageBuffer(r,c);
			count(i)=1;
		end
	end,	
end,

fid = mopen("output.txt", "w");
  if (fid == -1) then,
    error('cannot open file for writing');
  end
for k=1:i
	mfprintf(fid, "%d%c",color1(k),'a');
end,
mfprintf(fid,"%c",'b');
for k=1:i
	mfprintf(fid, "%d%c",count(k),'a');
end,
mfprintf(fid,"%c",'b');
mfprintf(fid,"%d",rows);
mfprintf(fid,"%c",'b');
mfprintf(fid,"%d",cols);
mfprintf(fid,"%c",'b');
mclose([fid]);
fod = mopen("output.txt", "r");
str = mgetstr(1000, [fod]);
ch = str2code(str);
n=length(str);
// Opening and cnfiguring the zigbee.
slLoad();			// Load the module
slMount(1)			// mount port 1
slCheck(1,port_no)		// assign COM9 serial port to port 1
slConfig(1,9600,8,0,1)		// configure the port.
slOpen(1,port_no)		// open COM9 serial port as port 1
for k=1:n
	slSendByte(1,ascii(code2str(ch(k))));	// Send ASCII data through that port.
end,
			
//disp(code2str(ch(1)));

	
