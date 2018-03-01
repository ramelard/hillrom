opencv custom format:

1) The Libraries and dll's given are built in VS2012.

To build the 10CUG camera for  32/64 bit register the "Bayer_Transform_Filter.dll" 
present in the respective folders (e-con_Modified_OpenCV\Windows\lib\<x32/x64>)


To register a filter:

1) Open the command prompt in Administrator mode.
2) Type regsvr32 <location of the filter with the filter name>  and press enter.



