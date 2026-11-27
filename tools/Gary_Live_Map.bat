@echo off
REM ---------------------------------------------------------------------------
REM Gary's live map -- double-click to launch.
REM
REM Starts gary_web_dashboard.py, which connects to Gary, asks him to turn
REM scanner mode on, and serves the live 3D map at http://localhost:8770
REM (your browser opens automatically after a second).
REM
REM If Gary's IP has moved (DHCP), change GARY_SOURCE below. His IP is printed
REM over serial at boot: ">>> WiFi connected. Scan server at tcp:<ip>:3333".
REM (Was .42 originally; DHCP moved him to .156 on 2026-07-12.)
REM To use the USB cable instead of WiFi, set GARY_SOURCE=COM15 (or whichever
REM port he shows up on in Device Manager).
REM
REM NOTE: the window now stays open on exit no matter what, so any error is
REM readable instead of vanishing with the console.
REM ---------------------------------------------------------------------------

set GARY_SOURCE=tcp:192.168.1.156:3333
set GARY_PORT=8770

cd /d "%~dp0"

echo.
echo   Folder : %CD%
echo   Source : %GARY_SOURCE%
echo   URL    : http://localhost:%GARY_PORT%
echo.

REM --- Find a working Python. "python" on PATH is the common case; the py
REM     launcher is the fallback Windows installs ship with.
set PY=python
where python >nul 2>&1
if errorlevel 1 (
  where py >nul 2>&1
  if errorlevel 1 (
    echo   [X] No Python found on PATH.
    echo       Install Python from python.org and tick "Add python.exe to PATH",
    echo       or edit this file and set PY to the full path of python.exe.
    echo.
    pause
    exit /b 1
  )
  set PY=py
)

echo   Using: %PY%
%PY% --version
echo.

REM --- Dependencies. numpy is required; pyserial only for the USB path.
%PY% -c "import numpy" 2>nul
if errorlevel 1 (
  echo   [!] numpy is missing. Installing numpy and pyserial ...
  %PY% -m pip install numpy pyserial
  echo.
)

%PY% gary_web_dashboard.py --source %GARY_SOURCE% --port %GARY_PORT%

echo.
echo   --- Dashboard exited. If that was unexpected, the error is above.
echo       Common causes:
echo         * port %GARY_PORT% already in use  ^(another copy still running?^)
echo         * python found but numpy/pyserial missing
echo         * Gary off or IP changed -- the PAGE still loads in that case,
echo           it just shows "NO DATA yet", so a browser "couldn't connect"
echo           error means THIS process died, not that Gary is unreachable.
echo.
pause
