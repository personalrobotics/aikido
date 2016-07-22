rem Build GLPK with Microsoft Visual C++ 6.0

rem NOTE: Make sure that HOME variable specifies correct path.
set HOME="C:\Program Files\Microsoft Visual Studio\VC98"

call %HOME%\bin\vcvars32.bat
copy config_VC6 config.h
%HOME%\bin\nmake.exe /f Makefile_VC6
%HOME%\bin\nmake.exe /f Makefile_VC6 check

pause
