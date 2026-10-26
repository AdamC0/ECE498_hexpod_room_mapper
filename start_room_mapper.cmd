@echo off
setlocal

set "ROBOT=hiwonder@192.168.137.223"
set "REMOTE_LAUNCHER=/home/hiwonder/room_mapper/start_room_mapper.sh"

echo Connecting to %ROBOT% and starting the room mapper safely...
echo The robot will remain AUTO PAUSED; this launcher does not begin motion.
echo.

ssh %ROBOT% "bash %REMOTE_LAUNCHER%"
if errorlevel 1 (
  echo.
  echo Startup failed. Review the message above and the robot logs.
  pause
  exit /b 1
)

echo.
echo Startup verified. Opening the mapping dashboard...
start "" "http://192.168.137.223:8000"
pause
