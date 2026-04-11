@echo off
REM ================================================================
REM SOMA Chess O1 - Windows bridge fallback attach via usbipd-win
REM
REM Use this only when you intentionally keep the PDP Xbox controller on
REM Windows and run scripts\bridge方案\bridge_gui.py. In this mode, only the RoArm
REM is attached to WSL. The C922 should stay on native Windows and publish
REM into WSL over ROS 2 DDS.
REM ================================================================

set ROARM_BUSID=1-4
echo Listing current usbipd state:
usbipd list
echo.

echo Attaching RoArm-M2-S (busid=%ROARM_BUSID%) to WSL...
usbipd attach --wsl --busid %ROARM_BUSID%

echo.
echo Done. In bridge fallback mode, keep the PDP controller on Windows and
echo launch scripts\bridge方案\bridge_gui.py there.
echo Keep Logitech C922 on Windows and run
echo scripts\bridge方案\start_camera_ros_publisher.bat there as needed.
echo.
pause
