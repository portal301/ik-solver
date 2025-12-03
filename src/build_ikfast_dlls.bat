@echo off
setlocal enabledelayedexpansion

rem 이 배치파일이 있는 위치 기준으로 경로 계산
set SCRIPT_DIR=%~dp0
rem 끝에 \ 가 붙어서 나와서 제거는 안 해도 됨

rem IKFast 소스가 들어있는 폴더 (src/robots)
set SRC_DIR=%SCRIPT_DIR%robots

rem ikfast.h 가 있는 include 폴더 (root/include)
set INCLUDE_DIR=%SCRIPT_DIR%..\include

rem 현재 VS 환경의 아키텍처 감지 (x86 또는 x64)
if "%VSCMD_ARG_TGT_ARCH%"=="" (
    echo VS Developer Command Prompt not detected. Setting up x64 environment...
    echo.

    rem VS 2022 설치 경로 찾기 (여러 에디션 시도)
    set "VCVARSALL="

    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VCVARSALL=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
        echo Found: Build Tools
    )
    if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
        echo Found: Community
    )
    if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat"
        echo Found: Professional
    )
    if exist "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat"
        echo Found: Enterprise
    )

    rem vcvarsall.bat 실행하여 x64 환경 설정
    if defined VCVARSALL (
        echo Calling: !VCVARSALL!
        call "!VCVARSALL!" x64
        set ARCH=x64
        echo VS 2022 x64 environment loaded successfully.
        echo.
    ) else (
        echo ERROR: Visual Studio 2022 not found!
        echo Please run this from "x64 Native Tools Command Prompt for VS 2022"
        echo Or install Visual Studio 2022 Build Tools
        pause
        exit /b 1
    )
) else (
    set ARCH=%VSCMD_ARG_TGT_ARCH%
)

echo Building for architecture: %ARCH%

rem vcpkg 라이브러리 경로 (아키텍처별)
set VCPKG_ROOT=C:\dev\vcpkg\installed\%ARCH%-windows
set VCPKG_INCLUDE=%VCPKG_ROOT%\include
set OPENBLAS_LIB=%VCPKG_ROOT%\lib\openblas.lib
set LAPACK_LIB=%VCPKG_ROOT%\lib\lapack.lib

rem 공통 컴파일 옵션
rem  - IKFAST_HAS_LIBRARY / IKFAST_CLIBRARY: ikfast.h 하단의 IKFAST_API 정의 활성화
rem  - /utf-8: UTF-8 인코딩으로 소스 파일 읽기
rem  - vcpkg include 경로 추가
set CL_OPTS=/nologo /std:c++17 /EHsc /LD /utf-8 ^
 /I"%INCLUDE_DIR%" /I"%VCPKG_INCLUDE%" ^
 /D IKFAST_HAS_LIBRARY /D IKFAST_CLIBRARY

rem 링크 옵션 (LAPACK + BLAS 라이브러리)
rem lapack-reference는 동적 라이브러리로 LAPACK 함수(dgetrf_, dgetrs_, dgeev_) 제공
set LINK_OPTS="%LAPACK_LIB%" "%OPENBLAS_LIB%"

rem SRC_DIR 안의 *_ikfast.cpp 전부에 대해 DLL 빌드
for %%F in ("%SRC_DIR%\*_ikfast.cpp") do (
    echo Building %%F ...
    cl %CL_OPTS% "%%F" /Fo:"%%~dpnF.obj" /Fe:"%%~dpnF.dll" /link %LINK_OPTS%
)

echo Done.
endlocal
rem pause