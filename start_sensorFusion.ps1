$envFilePath = ".\.env"

$content = Get-Content -Path $envFilePath

if (-Not $content) {
    Write-Output "Create a file with named '.env' (See README)"
    Exit
}

$RadonUlzerfilePathLine = $content | Where-Object { $_ -match '^RadonUlzer_RemoteControlSim_PATH=' }
if ($RadonUlzerfilePathLine) {
    $RadonUlzerFilePath = $RadonUlzerfilePathLine -split '=' | Select-Object -Last 1
    Write-Output "Der Dateipfad ist: $RadonUlzerFilePath"
} else {
    Write-Output "Key RadonUlzer_RemoteControlSim_PATH not found"
    Exit
}

$SpaceShipRadarfilePathLine = $content | Where-Object { $_ -match '^SpaceShipRadar_PATH=' }
if ($SpaceShipRadarfilePathLine) {
    $SpaceShipRadarFilePath = $SpaceShipRadarfilePathLine -split '=' | Select-Object -Last 1
    Write-Output "Der Dateipfad ist: $SpaceShipRadarFilePath"
} else {
    Write-Output "Key SpaceShipRadar_PATH not found"
    Exit
}

$DCSfilePathLine = $content | Where-Object { $_ -match '^DroidControlShip_PATH=' }
if ($DCSfilePathLine) {
    $DCSFilePath = $DCSfilePathLine -split '=' | Select-Object -Last 1
    Write-Output "Der Dateipfad ist: $DCSFilePath"
} else {
    Write-Output "Key DroidControlShip_PATH not found"
    Exit
}

# Pfad zum webots-controller
$webotsController = Join-Path $env:WEBOTS_HOME "msys64\mingw64\bin\webots-controller.exe"

# ===== RadonUlzer (Zumo / RemoteControlSim) ==================================
# Entspricht deinem PIO-Build-Aufruf:
# --robot-name=Zumo --stdout-redirect <RadonUlzerFilePath> -c --supervisorRxCh 1 ...
Start-Process -FilePath $webotsController `
    -ArgumentList @(
        "--robot-name=Zumo",
        "--stdout-redirect", $RadonUlzerFilePath,
        "-c",
        "--supervisorRxCh", "1",
        "--supervisorTxCh", "2",
        "--serialRxCh",     "3",
        "--serialTxCh",     "4",
        "--settingsPath",   "C:\Users\thaeckel\Documents\Repos\RadonUlzer\./settings/settings.json",
        "-v"
    )

# ===== SpaceShipRadar (MyBot) ================================================
Start-Process -FilePath $webotsController `
    -ArgumentList @(
        "--robot-name=MyBot",
        "--stdout-redirect", $SpaceShipRadarFilePath
    )

# ===== DroidControlShip / LineFollowerSensorFusionSim ========================
# Hier spiegeln wir den PIO-Aufruf:
# --robot-name=ZumoComSystem --stdout-redirect <DCSFilePath> \
# --cfgFilePath "../../../data/config/config.json" --serialRxCh 4 --serialTxCh 3 -v

# Arbeitsverzeichnis so setzen, dass der relative cfg-Pfad passt
$DCSWorkDir = Split-Path $DCSFilePath

Start-Process -FilePath $webotsController `
    -WorkingDirectory $DCSWorkDir `
    -ArgumentList @(
        "--robot-name=ZumoComSystem",
        "--stdout-redirect", $DCSFilePath,
        "--cfgFilePath", "../../../data/config/config_zumo1.json",
        "--serialRxCh", "4",
        "--serialTxCh", "3",
        "-v"
    )
