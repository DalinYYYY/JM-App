@echo off
echo Cleaning Keil MDK project temporary files...
echo.

rd /Q /S MDK-ARM\Objects 2>nul
rd /Q /S MDK-ARM\Listings 2>nul
rd /Q /S MDK-ARM\DebugConfig 2>nul
rd /Q /S MDK-ARM\JointMotorApp 2>nul
rd /Q /S MDK-ARM\RTE 2>nul

del /Q MDK-ARM\*.bak 2>nul
del /Q MDK-ARM\*.dep 2>nul
del /Q MDK-ARM\JLink* 2>nul
del /Q MDK-ARM\*.uvgui.* 2>nul
del /Q MDK-ARM\*.uvguix.* 2>nul

echo.
echo Cleanup completed successfully!
echo Please rebuild the entire project.
pause
