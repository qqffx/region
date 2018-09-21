@echo off

SET VC_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community
IF NOT DEFINED LIB (IF EXIST "%VC_PATH%" (call "%VC_PATH%\VC\Auxiliary\Build\vcvarsall.bat" x64))

set CommonLinkerFlags= -incremental:yes -opt:ref
IF NOT EXIST ..\build mkdir ..\build
pushd ..\build
del *.pdb > NUL 2> NUL

REM cl -Z7 -O2 -nologo -Fmsource.map ../code/source.c
cl -Z7 -nologo ../code/load_data.c
cl -Z7 -nologo ../code/analyze_data.c
popd

