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
if /i "%CMD%"=="python"   goto :python
if /i "%CMD%"=="hangar"   goto :hangar
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

:python
for %%I in ("%BIN%\..\python") do set "PYSTAGE=%%~fI"
if not exist "%PYSTAGE%\fsim-python.cmd" (
    echo The Python SDK is not built: it needs a CPython 3.11 or later from python.org or conda.
    echo Point the build at one and rebuild:
    echo.
    echo     cmake --preset ucrt64-release -DFSIM_PYTHON_EXECUTABLE=C:/path/to/python.exe
    echo     fsim build
    exit /b 1
)
rem `call`, not :run - a batch file run without it never returns here.
echo ^> "%PYSTAGE%\fsim-python.cmd" !ARGS!
echo.
call "%PYSTAGE%\fsim-python.cmd" !ARGS!
exit /b !errorlevel!

:hangar
rem The aircraft design tool runs on the Python the SDK was built for.
for %%I in ("%BIN%\..\python") do set "PYSTAGE=%%~fI"
if not exist "%PYSTAGE%\fsim-python.cmd" (
    echo hangar needs the Python SDK, which is not built: it needs a CPython 3.11 or later
    echo from python.org or conda, with numpy and matplotlib.
    exit /b 1
)
set "PYTHONPATH=%ROOT%tools\hangar;%PYTHONPATH%"
echo ^> "%PYSTAGE%\fsim-python.cmd" -m hangar !ARGS!
echo.
call "%PYSTAGE%\fsim-python.cmd" -m hangar !ARGS!
exit /b !errorlevel!

:where
echo applications  %BIN%
echo examples      %EXAMPLES%
echo tests         %TESTS%
for %%I in ("%BIN%\..\python") do echo python        %%~fI
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
echo     python\     the Python SDK: the fsim package, and a wheel to pip install
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
echo   AIRCRAFT OF YOUR OWN ^(tools\hangar, docs\hangar.md^)
echo     fsim hangar list                     the designs in aircraft\
echo     fsim hangar c172                     build and flight-test one: aircraft\c172\out\report.html
echo     fsim hangar c172 geometry            one stage ^(geometry aero mass propulsion build verify fly calibrate report^)
echo     fsim hangar skua --quick             every stage, coarse: a first look in half a minute
echo     fsim hangar new mine --like c172     start a design from another
echo     fsim demo --aircraft c172            watch it fly
echo     fsim python examples\python\control_surfaces.py   both designs flying a slalom; watch with
echo                                          fsim viewer --camera chase --chase-distance 20
echo.
echo   PYTHON
echo     fsim python examples\python\world_tour.py   the object model from Python
echo     fsim python examples\python\train_sb3.py    PPO ^(Stable-Baselines3^) on a 64-aircraft batch
echo     fsim python examples\python\train_plugin.py PPO on a task written in C++, loaded as a plugin DLL
echo     fsim python examples\python\speed.py        what the Python SDK costs, measured
echo     fsim python ^<script.py^>                     any script, with the fsim package on its path
echo.
echo   WORK ON IT
echo     fsim build             compile everything
echo     fsim test              run the test suite
echo     fsim dist              package the viewer, SDKs, tools and examples into dist\
echo     fetch-maps             download the map assets into assets\maps
echo     fsim where             print the directories things are built into
echo.
echo   Any extra arguments are passed straight through, e.g.
echo     fsim demo --vehicles 24 --lat 46.85 --lon 9.53
echo.
exit /b 0
