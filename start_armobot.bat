@echo off
cd /d "%~dp0"
echo =========================================
echo       Starting ARMOBOT Control System
echo =========================================
echo.

echo [1/3] Starting Backend Server (Port 3000)...
start "ARMOBOT Backend" cmd /k "cd Armobot-control-system\backend && npm start"

echo [2/3] Starting Frontend Server (Port 5173)...
start "ARMOBOT Frontend" cmd /k "cd Armobot-control-system\frontend && npm run dev"

echo.
echo [3/3] Waiting for servers to initialize...
timeout /t 5 /nobreak >nul

echo Opening browser to http://localhost:5173...
start http://localhost:5173

echo.
echo Done! You can close this small window. The backend and frontend terminals will remain open.
timeout /t 3 >nul
