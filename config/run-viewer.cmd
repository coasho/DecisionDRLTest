@echo off
rem Start the viewer from this package. Everything it needs - libraries,
rem configuration, map assets and the JSBSim data - is inside this directory.
setlocal
cd /d "%~dp0"
start "" "%~dp0bin\flightsim-viewer.exe" %*
