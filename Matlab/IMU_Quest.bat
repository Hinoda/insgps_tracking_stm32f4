set MATLAB=D:\MatlabR2009b
set MSVCDir=c:\program files\microsoft visual studio\vc98

set MSDevDir=c:\program files\microsoft visual studio\common\msdev98
"D:\MatlabR2009b\rtw\bin\win32\envcheck" INCLUDE "c:\program files\microsoft visual studio\vc98\include"
if errorlevel 1 goto vcvars32
"D:\MatlabR2009b\rtw\bin\win32\envcheck" PATH "c:\program files\microsoft visual studio\vc98\bin"
if errorlevel 1 goto vcvars32
goto make

:vcvars32
set VSCommonDir=c:\program files\microsoft visual studio\common
call "D:\MatlabR2009b\toolbox\rtw\rtw\private\vcvars32_600.bat"

:make
cd .
nmake -f IMU_Quest.mk  GENERATE_REPORT=0 GENERATE_ASAP2=0
@if errorlevel 1 goto error_exit
exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
