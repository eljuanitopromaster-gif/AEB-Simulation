@echo off
rem Corre un ensayo de la carpeta ensayos\ con el pedal C5-E: video en vivo de CARLA + ventana del pedal,
rem y al terminar abre las graficas y el video. Los resultados quedan en results\porsche\ensayos\<corrida>\.
rem Abre CARLA (%CARLA_ROOT%\CarlaUE4.exe) si no esta abierto.
rem Uso:  ensayo.bat                                            lista los ensayos
rem       ensayo.bat ENCAP_CCRb_6_12
rem       ensayo.bat ENCAP_CCRb_6_12 --clima tormenta_noche --sin-trafico
rem       ensayo.bat MI_ESCENARIO_PEATON_30KMH --clima lluvia
rem Opciones: las de run_scenario.py (venv\Scripts\python.exe run_scenario.py --help)
setlocal
cd /d "%~dp0"
for /f "tokens=2 delims=:" %%a in ('chcp') do set "CP_ORIGINAL=%%a"
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"

if "%~1"=="" (
    echo Ensayos disponibles:
    for %%f in (ensayos\*.py) do echo    %%~nf
    echo.
    echo Uso: ensayo.bat NOMBRE [--clima despejado^|lluvia^|niebla^|tormenta_noche] [--sin-trafico] [--carpeta NOMBRE]
    goto :fin_error
)
if not exist "ensayos\%~1.py" (
    echo No existe ensayos\%~1.py
    goto :fin_error
)
if not exist "venv\Scripts\python.exe" (
    echo Falta el venv: corre primero instalar.bat
    goto :fin_error
)

powershell -NoProfile -Command "function Up { try { $c = New-Object Net.Sockets.TcpClient; $c.Connect('localhost', 2000); $c.Close(); $true } catch { $false } }; if (Up) { exit 0 }; if (-not $env:CARLA_ROOT) { Write-Host 'CARLA no esta abierto y falta CARLA_ROOT: abre CarlaUE4.exe a mano o define CARLA_ROOT.'; exit 1 }; Write-Host 'Abriendo CARLA...'; Start-Process (Join-Path $env:CARLA_ROOT 'CarlaUE4.exe') -ArgumentList '-windowed','-ResX=1280','-ResY=720','-quality-level=Low'; while (-not (Up)) { Start-Sleep 2 }; Start-Sleep 30"
if errorlevel 1 goto :fin_error

"venv\Scripts\python.exe" run_scenario.py %* --ver --abrir
if errorlevel 1 (
    echo La simulacion fallo.
    goto :fin_error
)
chcp %CP_ORIGINAL% >nul
exit /b 0

:fin_error
chcp %CP_ORIGINAL% >nul
exit /b 1
