@echo off
rem PEO Rover upper computer launcher (double-click to run)
cd /d "%~dp0"
"C:\Users\Danny\AppData\Local\Programs\Python\Python313\python.exe" -m app.main
if errorlevel 1 (
    echo.
    echo Startup failed, check the errors above.
    pause
)
