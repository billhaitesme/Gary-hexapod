@echo off
REM ---------------------------------------------------------------------------
REM Run this ONCE. It puts a "Gary Live Map" shortcut on your Desktop that
REM launches the live map dashboard (Gary_Live_Map.bat in this folder).
REM
REM Uses PowerShell's WScript.Shell to write a real .lnk -- so it gets a proper
REM icon and can be pinned to the taskbar / Start like any other app.
REM ---------------------------------------------------------------------------

setlocal
set TARGET=%~dp0Gary_Live_Map.bat
set ICON=%SystemRoot%\System32\SHELL32.dll,13

powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$s = (New-Object -ComObject WScript.Shell).CreateShortcut([Environment]::GetFolderPath('Desktop') + '\Gary Live Map.lnk');" ^
  "$s.TargetPath = '%TARGET%';" ^
  "$s.WorkingDirectory = '%~dp0';" ^
  "$s.IconLocation = '%ICON%';" ^
  "$s.Description = 'Gary: live 3D LiDAR map + CSI presence monitor';" ^
  "$s.Save()"

if errorlevel 1 (
  echo.
  echo   Could not create the shortcut. You can make one by hand:
  echo   right-click Gary_Live_Map.bat -^> Send to -^> Desktop ^(create shortcut^)
  echo.
) else (
  echo.
  echo   Done -- "Gary Live Map" is on your Desktop.
  echo   Double-click it any time to bring the map up.
  echo.
)

pause
endlocal
