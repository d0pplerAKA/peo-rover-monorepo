@echo off
rem PEO Rover 上位机启动脚本（双击即可运行）
cd /d "%~dp0"
"C:\Users\Danny\AppData\Local\Programs\Python\Python313\python.exe" -m app.main
if errorlevel 1 (
    echo.
    echo 启动失败，请检查上方错误信息。
    pause
)
