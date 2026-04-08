@echo off
REM ================================================================
REM SmartRobotArm - WSL2 USB device forwarding via usbipd-win
REM
REM Run this in Windows PowerShell (as Administrator) every time you
REM reboot Windows. It attaches the RoArm-M2-S serial bridge and the
REM PDP Xbox gamepad to the WSL2 kernel so ROS 2 can see them as
REM /dev/ttyUSB0 and /dev/input/js0.
REM
REM First-time setup only (once per Windows install):
REM     usbipd list
REM     usbipd bind --busid <ROARM_BUSID>
REM     usbipd bind --busid <PDP_BUSID>
REM
REM Every reboot:
REM     attach_devices.bat
REM ================================================================

REM === EDIT THESE TWO BUSIDs after the first `usbipd list` ===
set ROARM_BUSID=1-2
set PDP_BUSID=1-3
REM ===========================================================

echo Listing current usbipd state:
usbipd list
echo.

echo Attaching RoArm-M2-S (busid=%ROARM_BUSID%) to WSL...
usbipd attach --wsl --busid %ROARM_BUSID%

echo Attaching PDP Xbox gamepad (busid=%PDP_BUSID%) to WSL...
usbipd attach --wsl --busid %PDP_BUSID%

echo.
echo Done. Inside WSL you should now see:
echo   /dev/ttyUSB0   (RoArm-M2-S)
echo   /dev/input/js0 (PDP Xbox gamepad)
echo.
pause
