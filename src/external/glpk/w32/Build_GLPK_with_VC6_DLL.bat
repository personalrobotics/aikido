rem Build GLPK DLL with Microsoft Visual C++ 6.0

rem NOTE: Make sure that HOME variable specifies correct path.
set HOME="C:\Program Files\Microsoft Visual Studio\VC98"

call %HOME%\bin\vcvars32.bat
copy config_VC6_DLL config.h
%HOME%\bin\nmake.exe /f Makefile_VC6_DLL
%HOME%\bin\nmake.exe /f Makefile_VC6_DLL check

pause
