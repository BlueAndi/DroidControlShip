@echo off
setlocal ENABLEEXTENSIONS

rem -----------------------------------------------------------
rem Usage:
rem   start_sensorFusion.bat [zumoId]
rem   zumoId: 1..5 (Default: 1)
rem
rem Starts:
rem   - RadonUlzer RemoteControlSim  (robot-name=Zumo)
rem   - DCS LineFollowerSensorFusionSim (robot-name=ZumoComSystem)
rem
rem LineFollowerSensorFusionSim is started equivalent to the
rem webots_launcher.py (with cfgFilePath + serialRx/Tx).
rem -----------------------------------------------------------

set "SF_DEBUG=1"

rem --- Clean up possible leftover Webots controllers / native controllers from previous runs ---
taskkill /IM program.exe /F >nul 2>&1
taskkill /IM webots-controller.exe /F >nul 2>&1



rem === Default Zumo-ID = 1 ===
set "ZUMO_ID=%1"
if "%ZUMO_ID%"=="" set "ZUMO_ID=1"

rem === Validate range 1..5 ===
if %ZUMO_ID% LSS 1 goto invalid_zumo
if %ZUMO_ID% GTR 5 goto invalid_zumo

echo ==========================================
echo  Starting Sensor Fusion for Zumo %ZUMO_ID%
echo  Debug mode: %SF_DEBUG%
echo ==========================================
echo.

rem === Check WEBOTS_HOME ===
if "%WEBOTS_HOME%"=="" (
    echo [ERROR] WEBOTS_HOME is not set.
    echo         Please set WEBOTS_HOME to your Webots installation directory.
    echo         Example: set WEBOTS_HOME=C:\Program Files\Webots
    goto error_exit
)

rem === Paths / tools ===
set "WEBOTS_CONTROLLER=%WEBOTS_HOME%\msys64\mingw64\bin\webots-controller.exe"
set "PROGRAM_NAME=program.exe"

if not exist "%WEBOTS_CONTROLLER%" (
    echo [ERROR] webots-controller.exe not found:
    echo         %WEBOTS_CONTROLLER%
    goto error_exit
)

rem === Root paths ===
set "DCS_ROOT=%~dp0"
set "RADON_ROOT=%~dp0..\RadonUlzer"

rem === Config file for Zumo (DCS-side) ===
set "CONFIG_FILE=config_zumo%ZUMO_ID%.json"
set "DCS_CONFIG_PATH=../../../data/config/%CONFIG_FILE%"

rem === Build paths for executables ===
set "RADON_PROGRAM_PATH=%RADON_ROOT%\.pio\build\RemoteControlSim"
set "DCS_PROGRAM_PATH=%DCS_ROOT%\.pio\build\LineFollowerSensorFusionSim"

if "%SF_DEBUG%"=="1" (
    echo [DEBUG] WEBOTS_CONTROLLER  = %WEBOTS_CONTROLLER%
    echo [DEBUG] RADON_ROOT         = %RADON_ROOT%
    echo [DEBUG] RADON_PROGRAM_PATH = %RADON_PROGRAM_PATH%
    echo [DEBUG] DCS_ROOT           = %DCS_ROOT%
    echo [DEBUG] DCS_PROGRAM_PATH   = %DCS_PROGRAM_PATH%
    echo [DEBUG] DCS_CONFIG_PATH    = %DCS_CONFIG_PATH%
    echo.
)

rem -----------------------------------------------------------
rem 1) RadonUlzer: build & start RemoteControlSim
rem -----------------------------------------------------------
pushd "%RADON_ROOT%"
echo [RadonUlzer] Building RemoteControlSim ...
"%USERPROFILE%\.platformio\penv\Scripts\pio.exe" run --environment RemoteControlSim
if errorlevel 1 (
    echo [RadonUlzer] Build failed. Aborting.
    popd
    goto error_exit
)

echo [RadonUlzer] Starting RemoteControlSim for Zumo %ZUMO_ID% ...
echo [DEBUG] Command:
echo   "%WEBOTS_CONTROLLER%" --robot-name=Zumo --stdout-redirect "%RADON_PROGRAM_PATH%\%PROGRAM_NAME%" -c --supervisorRxCh 1 --supervisorTxCh 2 --serialRxCh 3 --serialTxCh 4 --settingsPath "%RADON_ROOT%\settings\settings.json" -v

start "" /B ^
  "%WEBOTS_CONTROLLER%" ^
  --robot-name=Zumo ^
  --stdout-redirect "%RADON_PROGRAM_PATH%\%PROGRAM_NAME%" ^
  -c ^
  --supervisorRxCh 1 ^
  --supervisorTxCh 2 ^
  --serialRxCh 3 ^
  --serialTxCh 4 ^
  --settingsPath "%RADON_ROOT%\settings\settings.json" ^
  -v

popd
echo.

rem -----------------------------------------------------------
rem 2) DCS: build & start LineFollowerSensorFusionSim
rem     → exakt wie webots_launcher.py, nur mit parametrisierter config_zumoX
rem -----------------------------------------------------------
pushd "%DCS_ROOT%"
echo [DCS] Building LineFollowerSensorFusionSim ...
"%USERPROFILE%\.platformio\penv\Scripts\pio.exe" run --environment LineFollowerSensorFusionSim
if errorlevel 1 (
    echo [DCS] Build failed. Aborting.
    popd
    goto error_exit
)

echo [DCS] Starting LineFollowerSensorFusionSim for Zumo %ZUMO_ID% ...

rem =======================
rem Webots launcher pattern
rem =======================
rem On Windows:
rem   WEBOTS_CONTROLLER = ".../webots-controller.exe"
rem   ROBOT_NAME        = ZumoComSystem (from platformio.ini: webots_robot_name)
rem   PROGRAM           = .pio\build\LineFollowerSensorFusionSim\program.exe
rem   PROGRAM_OPTIONS   = --cfgFilePath "../../../data/config/config_zumoX.json"
rem                       --serialRxCh <rx>
rem                       --serialTxCh <tx>
rem                       -v
rem Everything AFTER program.exe is passed to program.exe (not to Webots).

set "DCS_ROBOT_NAME=ZumoComSystem"
set "DCS_SERIAL_RX_CH=4"
set "DCS_SERIAL_TX_CH=3"

if "%SF_DEBUG%"=="1" (
    echo [DEBUG] Command (DCS)
    echo   "%WEBOTS_CONTROLLER%" --robot-name=%DCS_ROBOT_NAME% --stdout-redirect "%DCS_PROGRAM_PATH%\%PROGRAM_NAME%" --cfgFilePath "%DCS_CONFIG_PATH%" --serialRxCh %DCS_SERIAL_RX_CH% --serialTxCh %DCS_SERIAL_TX_CH% -v
    echo.
)

"%WEBOTS_CONTROLLER%" --robot-name=%DCS_ROBOT_NAME% --stdout-redirect "%DCS_PROGRAM_PATH%\%PROGRAM_NAME%" --cfgFilePath "%DCS_CONFIG_PATH%" --serialRxCh %DCS_SERIAL_RX_CH% --serialTxCh %DCS_SERIAL_TX_CH% -v

echo.
echo [DCS] LineFollowerSensorFusionSim terminated (see output above).
popd
echo.

echo ==========================================
echo     Script finished (see above output)
echo ==========================================
if "%SF_DEBUG%"=="1" (
    echo.
    echo Press any key to close this window...
    pause >nul
)

endlocal
exit /b 0

:invalid_zumo
echo [ERROR] Invalid Zumo ID "%ZUMO_ID%". Use 1..5.
pause
endlocal
exit /b 1

:error_exit
echo.
echo Script aborted due to errors.
endlocal
exit /b 2
