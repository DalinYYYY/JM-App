@echo off
echo Cleaning Keil MDK project temporary files...
echo.

rd /Q /S Board\V1\MDK-ARM\Objects 2>nul
rd /Q /S Board\V1\MDK-ARM\Listings 2>nul
rd /Q /S Board\V1\MDK-ARM\DebugConfig 2>nul
rd /Q /S Board\V1\MDK-ARM\JointMotorApp 2>nul
rd /Q /S Board\V1\MDK-ARM\RTE 2>nul

del /Q Board\V1\MDK-ARM\*.bak 2>nul
del /Q Board\V1\MDK-ARM\*.dep 2>nul
del /Q Board\V1\MDK-ARM\JLink* 2>nul
del /Q Board\V1\MDK-ARM\*.uvgui.* 2>nul
del /Q Board\V1\MDK-ARM\*.uvguix.* 2>nul

echo.
echo Cleanup completed successfully!
echo Please rebuild the entire project.
pause
