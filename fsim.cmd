@echo off
rem flightsim launcher: one place to run everything, from the repository root.
rem The executables live in a build directory; this finds them and shows the
rem full command it runs, so you can copy it if you want to run it directly.
setlocal enabledelayedexpansion

set "ROOT=%~dp0"
set "BIN="
for %%D in ("build\ucrt64-release\bin" "build\ucrt64-headless\bin" "dist\flightsim\bin" "build\ucrt64-debug\bin") do (
    if not defined BIN if exist "%ROOT%%%~D\flightsim-viewer.exe" set "BIN=%ROOT%%%~D"
    if not defined BIN if exist "%ROOT%%%~D\flightsim.exe" set "BIN=%ROOT%%%~D"
)
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
if /i "%CMD%"=="train"    call :run "%BIN%\ppo_trainer.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="fly"      call :run "%BIN%\minimal_trainer.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="control"  call :run "%BIN%\multi_level_control.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="scenario" call :run "%BIN%\scenario_runner.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="cameras"  call :run "%BIN%\vision_capture.exe" --segmentation !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="headless" call :run "%BIN%\flightsim.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="tiles"    call :run "%BIN%\tile_prefetch.exe" !ARGS! & exit /b !errorlevel!
if /i "%CMD%"=="where"    echo %BIN% & exit /b 0

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
echo     fsim where             print the directory the executables are in
echo.
echo   Any extra arguments are passed straight through, e.g.
echo     fsim demo --vehicles 24 --lat 46.85 --lon 9.53
echo.
exit /b 0
