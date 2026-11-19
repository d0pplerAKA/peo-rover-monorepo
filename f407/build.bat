@echo off
cd /d "C:\Users\Danny\Documents\Keil Files\Keil Project\peo_rover"
"C:\Users\Danny\AppData\Local\Keil_v5\UV4\UV4.exe" -b "peo_rover_f407.uvprojx" -j0 -o "build_log.txt"
echo UV4_EXIT=%ERRORLEVEL%
