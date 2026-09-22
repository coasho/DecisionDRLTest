@echo off
rem flightsim launcher: one place to run everything, from the repository root.
rem The executables live in a build directory; this finds them and shows the
rem full command it runs, so you can copy it if you want to run it directly.
setlocal enabledelayedexpansion

set "ROOT=%~dp0"
set "BIN="
for %%D in ("build\ucrt64-release\bin" "build\ucrt64-headless\bin" "build\ucrt64-release\dist\viewer\bin" "build\ucrt64-debug\bin") do (
    if not defined BIN if exist "%ROOT%%%~D\flightsim-viewer.exe" set "BIN=%ROOT%%%~D"
    if not defined BIN if exist "%ROOT%%%~D\flightsim.exe" set "BIN=%ROOT%%%~D"
)
rem Applications, example trainers and tests are built into separate
rem directories - see cmake/Install.cmake. The examples sit next to bin/.
for %%I in ("%BIN%\..\examples") do set "EXAMPLES=%%~fI"
if not exist "%EXAMPLES%\ppo_trainer.exe" set "EXAMPLES=%BIN%"
for %%I in ("%BIN%\..\tests") do set "TESTS=%%~fI"
if not defined BIN (
    echo Nothing is built yet.
    echo.
    echo     fsim build          build everything ^(a few minutes the first time^)
    echo.
    exit /b 1
)

set "CMD=%~1"
set "ARGS=%*"
if defined ARGS if defined CMD set "ARGS=!ARGS:*%1=!"

if "%CMD%"=="" goto :menu
if /i "%CMD%"=="help" goto :menu
if /i "%CMD%"=="--help" goto :menu
if /i "%CMD%"=="-h" goto :menu

if /i "%CMD%"=="demo"     call :run "%BIN%\flightsim-viewer.exe" --demo --vehicles 6 !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="viewer"   call :run "%BIN%\flightsim-viewer.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="replay"   call :run "%BIN%\flightsim-viewer.exe" --replay !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="train"    call :run "%EXAMPLES%\ppo_trainer.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="fly"      call :run "%EXAMPLES%\minimal_trainer.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="control"  call :run "%EXAMPLES%\multi_level_control.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="scenario" call :run "%EXAMPLES%\scenario_runner.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="cameras"  call :run "%EXAMPLES%\vision_capture.exe" --segmentation !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="headless" call :run "%BIN%\flightsim.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="tiles"    call :run "%BIN%\tile_prefetch.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="where"    goto :where
if /i "%CMD%"=="dist"     goto :dist

if /i "%CMD%"=="build" (
    echo ^> cmake --build --preset ucrt64-release
    cmake --build --preset ucrt64-release
    exit /b !errorlevel!
)
if /i "%CMD%"=="test" (
    echo ^> ctest --preset ucrt64-release
    ctest --preset ucrt64-release
    exit /b !errorlevel!
)

echo Unknown command "%CMD%".
echo.
goto :menu

:run
echo ^> %*
echo.
%*
exit /b %errorlevel%

:where
echo applications  %BIN%
echo examples      %EXAMPLES%
echo tests         %TESTS%
echo packages      %ROOT%build\ucrt64-release\dist
exit /b 0

:dist
echo ^> cmake --build build/ucrt64-release --target dist
cmake --build build/ucrt64-release --target dist
if errorlevel 1 exit /b 1
echo.
echo Packaged into build\ucrt64-release\dist:
echo     viewer\     the visualisation application - run viewer\run-viewer.cmd
echo     sdk\        fsim.dll, headers and the CMake package
echo     tools\      the headless command-line application
echo     examples\   demo trainers against the SDK
exit /b 0

:menu
echo.
echo   flightsim - a flight simulator for reinforcement learning
echo.
echo   SEE IT
echo     fsim demo              fly 6 aircraft over San Francisco ^(start here^)
echo     fsim cameras           render what an aircraft's cameras see, as PNGs
echo     fsim viewer            watch a training run that is already going
echo.
echo   RUN IT
echo     fsim train             train a policy to hold altitude and heading
echo     fsim fly               a hand-written controller, no learning
echo     fsim control           the six control levels, one after another
echo     fsim scenario ^<file^>   run a scenario file ^(examples\scenarios\*.json^)
echo     fsim replay ^<file^>     play back a recording ^(.fsrec^)
echo.
echo   WORK ON IT
echo     fsim build             compile everything
echo     fsim test              run the test suite
echo     fsim dist              package the viewer, SDK, tools and examples into dist\
echo     fetch-maps             download the map assets into assets\maps
echo     fsim where             print the directories things are built into
echo.
echo   Any extra arguments are passed straight through, e.g.
echo     fsim demo --vehicles 24 --lat 46.85 --lon 9.53
echo.
exit /b 0
