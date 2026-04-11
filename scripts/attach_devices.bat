@echo off
REM ================================================================
REM SOMA Chess O1 - WSL-direct USB attach via usbipd-win
REM
REM Run this in Windows PowerShell (as Administrator) every time you
REM reboot Windows. It attaches the RoArm-M2-S serial bridge and the
REM PDP Xbox gamepad directly into WSL so ROS 2 can see them as
REM /dev/ttyUSB* and /dev/input/event*.
REM
REM IMPORTANT:
REM   - This is the recommended long-term workflow for arm + gamepad.
REM   - Your WSL kernel must have EVDEV + JOYDEV + XPAD enabled, otherwise the
REM     controller may appear in lsusb but /dev/input/event0 may not show up.
REM   - While the controller is attached to WSL, Windows can no longer use it.
REM   - As of 2026-04-10, the C922 is no longer attached here by default.
REM     Keep the camera on native Windows and publish it into WSL over ROS 2.
REM
REM First-time setup only (once per Windows install):
REM     usbipd list
REM     usbipd bind --busid <ROARM_BUSID>
REM     usbipd bind --busid <PDP_BUSID>
REM     usbipd bind --busid <C922_BUSID>   (debug-only; old WSL-direct camera path)
REM
REM Every reboot:
REM     attach_devices.bat
REM ================================================================

REM === Current recorded busids on this machine (2026-04-10) ===
set ROARM_BUSID=1-4
set PDP_BUSID=1-8
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
echo   /dev/ttyUSB*      (RoArm-M2-S; current node may be ttyUSB0 or ttyUSB1)
echo   /dev/input/event0 (PDP Xbox gamepad; default teleop path)
echo   /dev/input/js0    (optional legacy joydev node)
echo.
echo If /dev/input/event0 does not appear, verify your custom WSL kernel
echo enables CONFIG_INPUT_EVDEV and CONFIG_JOYSTICK_XPAD.
echo.
echo Preferred launch inside WSL:
echo   scripts/start_teleop_wsl_gamepad.sh
echo.
echo Camera note:
echo   Keep Logitech C922 on native Windows.
echo   Launch scripts\bridge方案\start_camera_ros_publisher.bat there.
echo.
pause
