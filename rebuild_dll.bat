@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d C:\dev\ikfast-generator\ik-solver\src
echo Rebuilding IKFastUnity_x64.dll...
cl /nologo /std:c++17 /EHsc /LD /utf-8 /O2 ^
   /I"..\include" ^
   /D IKFAST_HAS_LIBRARY ^
   ikfast_core.cpp ikfast_unity.cpp ^
   /Fe:"..\bin\IKFastUnity_x64.dll" ^
   /Fo:"..\bin\\"
if %ERRORLEVEL% neq 0 (
    echo Build failed!
    exit /b 1
)
echo Build successful!
