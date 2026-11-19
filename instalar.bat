@echo off
rem Instala el proyecto en esta carpeta: crea venv\ con Python 3.12 e instala las librerías y la API de CARLA.
rem Antes: instala Python 3.12 (con el lanzador "py") y CARLA 0.9.16, y define CARLA_ROOT, por ejemplo:
rem     setx CARLA_ROOT "C:\CARLA_0.9.16"      (y abre una consola nueva)
setlocal
cd /d "%~dp0"
set "PYTHONIOENCODING=utf-8"

if "%CARLA_ROOT%"=="" (
    echo Falta la variable CARLA_ROOT con la carpeta de CARLA 0.9.16 ^(la que tiene CarlaUE4.exe^).
    echo Ejemplo:  setx CARLA_ROOT "C:\CARLA_0.9.16"   y vuelve a abrir la consola.
    exit /b 1
)
set "WHEEL=%CARLA_ROOT%\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
if not exist "%WHEEL%" (
    echo No encuentro %WHEEL%
    echo Revisa que CARLA_ROOT apunte a CARLA 0.9.16.
    exit /b 1
)

if not exist "venv\Scripts\python.exe" (
    echo Creando venv con Python 3.12...
    py -3.12 -m venv venv || (echo No se pudo crear el venv: instala Python 3.12. & exit /b 1)
)
"venv\Scripts\python.exe" -m pip install --upgrade pip
"venv\Scripts\python.exe" -m pip install -r requirements.txt "%WHEEL%" || exit /b 1

echo.
echo Probando el núcleo del AEB...
"venv\Scripts\python.exe" -m unittest test_core || exit /b 1
echo.
echo Listo. Abre CarlaUE4.exe y corre, por ejemplo:
echo     venv\Scripts\python.exe run_scenario.py ENCAP_CCRb_6_12 --clima lluvia --ver --abrir
exit /b 0
