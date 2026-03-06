@echo off
REM Build SUMO net file from plain nodes/edges (Windows)

if "%SUMO_HOME%"=="" (
  echo [ERROR] SUMO_HOME is not set.
  echo Example:
  echo   set SUMO_HOME=D:\sim\sumo-1.8.0
  exit /b 1
)

if not exist "%SUMO_HOME%\bin\netconvert.exe" (
  echo [ERROR] netconvert.exe not found under %SUMO_HOME%\bin
  exit /b 1
)

echo Building net\net.net.xml ...
"%SUMO_HOME%\bin\netconvert.exe" ^
  --node-files net\nodes.nod.xml ^
  --edge-files net\edges.edg.xml ^
  -o net\net.net.xml

if errorlevel 1 (
  echo [ERROR] netconvert failed.
  exit /b 1
)

echo Done.
