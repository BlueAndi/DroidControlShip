@echo on

set PATH=%PATH%;
set WEBOTS_CONTROLLER="%WEBOTS_HOME%\msys64\mingw64\bin\webots-controller.exe"
set PROGRAM_NAME=program.exe
set CONVOY_LEADER_PROGRAM_PATH=.pio\build\ConvoyLeaderSim
set CONVOY_FOLLOWER_PROGRAM_PATH=.pio\build\ConvoyFollowerSim

rem Compile Convoy Leader
%USERPROFILE%\.platformio\penv\Scripts\pio.exe run --environment ConvoyLeaderSim

rem Compile Convoy Follower
%USERPROFILE%\.platformio\penv\Scripts\pio.exe run --environment ConvoyFollowerSim

rem Start the convoy leader
echo Start convoy leader.
start "ZumoComSystem Convoy Leader" ""%WEBOTS_CONTROLLER%"" --robot-name=ZumoComSystemLeader --stdout-redirect %CONVOY_LEADER_PROGRAM_PATH%\%PROGRAM_NAME% --cfgFilePath "../../../data/config/leader.json" --serialRxCh=2 --serialTxCh=1 -v"

rem Start the followers
for /L %%i in (1, 1, 2) do (
    call :start_follower %%i
)

exit /b

:start_follower
set INSTANCE=%1
set ROBOT_NAME=ZumoComSystemFollower%instance%
set /a SERIAL_RX_CHANNEL=(INSTANCE * 2) + 1
set /a SERIAL_TX_CHANNEL=(INSTANCE * 2) + 0
echo Start convoy follower %INSTANCE%.
start "ZumoComSystem Convoy Follower %INSTANCE%" ""%WEBOTS_CONTROLLER%"" --robot-name=%ROBOT_NAME% --stdout-redirect %CONVOY_FOLLOWER_PROGRAM_PATH%\%PROGRAM_NAME% --cfgFilePath "../../../data/config/follower_%instance%.json" --serialRxCh=%SERIAL_RX_CHANNEL% --serialTxCh=%SERIAL_TX_CHANNEL% -v"
exit /b
