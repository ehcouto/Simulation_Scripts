@echo off
echo === PMSM FOC Scope Launcher ===
echo.

:: Paths
set PYTHON=D:\Tools\MSYS\ucrt64\bin\python3.exe
set EXE=%~dp0Debug\MotorControlSimulation.exe
set SCOPE=%~dp0scope.py

:: Check python
echo [1] Verificando Python...
if not exist "%PYTHON%" (
    echo ERRO: Python nao encontrado em %PYTHON%
    goto error
)
echo OK: %PYTHON%

:: Check exe
echo [2] Verificando executavel...
if not exist "%EXE%" (
    echo ERRO: Executavel nao encontrado em %EXE%
    goto error
)
echo OK: %EXE%

:: Check scope
echo [3] Verificando scope.py...
if not exist "%SCOPE%" (
    echo ERRO: scope.py nao encontrado em %SCOPE%
    goto error
)
echo OK: %SCOPE%

:: Run
echo.
echo [4] Iniciando simulacao + scope...
"%EXE%" | "%PYTHON%" "%SCOPE%"
goto end

:error
echo.
echo Corrija o erro acima e tente novamente.

:end
pause