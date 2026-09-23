@echo off
rem Download the map assets this project draws the Earth from.
rem
rem They are gigabytes of imagery and elevation tiles, so they are not in git.
rem This fetches them into assets\maps, which is where the build copies them
rem from into a distributed package. Re-running is safe and cheap: tiles
rem already present are skipped, so an interrupted download resumes.
rem
rem   fetch-maps              download what the plan asks for (about 2.4 GB)
rem   fetch-maps --dry-run    count and price it first, download nothing
rem   fetch-maps --prune      also delete tiles the plan no longer wants
rem
rem What gets downloaded is assets\config\offline-map-plan.json: a global base,
rem every continent to level 9, mountain ranges with relief to level 11 and
rem imagery to 10 or 11, airports with imagery to level 14, and route
rem corridors. Edit that file to change where and how much.

setlocal
set "ROOT=%~dp0"
set "PLAN=%ROOT%assets\config\offline-map-plan.json"
set "MAPS=%ROOT%assets\maps"

set "TOOL="
for %%D in ("build\ucrt64-release\bin" "build\ucrt64-headless\bin" "build\ucrt64-debug\bin") do (
    if not defined TOOL if exist "%ROOT%%%~D\tile_prefetch.exe" set "TOOL=%ROOT%%%~D\tile_prefetch.exe"
)
if not defined TOOL (
    echo tile_prefetch is not built yet.
    echo.
    echo     fsim build
    echo.
    exit /b 1
)
if not exist "%PLAN%" (
    echo No map plan at %PLAN%
    exit /b 1
)

echo ^> "%TOOL%" --plan "%PLAN%" --cache "%MAPS%" --threads 12 %*
"%TOOL%" --plan "%PLAN%" --cache "%MAPS%" --threads 12 %*
set "RC=%ERRORLEVEL%"
if not "%RC%"=="0" (
    echo.
    echo Some tiles failed. Run this again - what arrived is kept and the rest retried.
    exit /b %RC%
)
echo.
echo Map assets are in %MAPS%
echo They are copied into the package by: fsim dist
exit /b 0
