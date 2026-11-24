@echo off
rem Windows: build one act program for the PC and run the scenarios.
rem usage: .\run.bat [program] [scenario | all]
rem   .\run.bat                              -> act_defense_smach, all scenarios
rem   .\run.bat act_defense_smach ball_front -> one scenario, opens it in the browser
rem   .\run.bat live                         -> live mode in the browser (drag the ball)
rem   .\run.bat act_attack live              -> live mode, attack program
rem Output: sim\out\<program>_<scenario>.html
setlocal
cd /d "%~dp0"

set PROG=%~1
if /i "%PROG%"=="live" (set PROG=act_defense_smach& set LIVEARG=live)
if "%PROG%"=="" set PROG=act_defense_smach
set SCEN=%~2
if defined LIVEARG set SCEN=live
if "%SCEN%"=="" set SCEN=all

rem defense robot: mux 2 line sensors
set MUX2=0
echo %PROG% | findstr /i "defense" >nul && set MUX2=1

rem g++ not on PATH (new install, old terminal): look in the usual install folders
where g++ >nul 2>nul
if errorlevel 1 call :findgpp
where g++ >nul 2>nul
if errorlevel 1 (
  echo g++ not found. Install it once, then open a NEW terminal:
  echo     winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT --source winget
  echo Check if it is installed:  winget list WinLibs
  exit /b 1
)

if not exist out mkdir out
echo Building %PROG% ...
g++ -std=gnu++17 -O2 -w -fpermissive -static -Istub -I../lib/core_act -I../include ^
    -DSIM_MUX2=%MUX2% -DSIM_PROG=\"%PROG%\" ^
    ../lib/core_act/core_act.cpp ../src/%PROG%.cpp fake_hw.cpp world.cpp live.cpp -lws2_32 -o out/sim_%PROG%.exe
if errorlevel 1 exit /b 1

out\sim_%PROG%.exe %SCEN%
if not "%SCEN%"=="all" if exist "out\%PROG%_%SCEN%.html" start "" "out\%PROG%_%SCEN%.html"
exit /b 0

:findgpp
for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs*") do if exist "%%D\mingw64\bin\g++.exe" set "PATH=%%D\mingw64\bin;%PATH%"
for %%P in ("C:\msys64\ucrt64\bin" "C:\msys64\mingw64\bin" "C:\mingw64\bin" "C:\MinGW\bin") do if exist "%%~P\g++.exe" set "PATH=%%~P;%PATH%"
where g++ >nul 2>nul && (echo Using g++ from: & where g++)
exit /b 0
