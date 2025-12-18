#requires -Version 5.1
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# =============================================================================
# Helpers
# =============================================================================
function Find-Upwards {
    param(
        [Parameter(Mandatory)][string]$StartDir,
        [Parameter(Mandatory)][string]$Target
    )

    $dir = (Resolve-Path $StartDir).Path
    while ($true) {
        $candidate = Join-Path $dir $Target
        if (Test-Path -LiteralPath $candidate) { return (Resolve-Path $candidate).Path }

        $parent = Split-Path -Parent $dir
        if ($parent -eq $dir) { break }
        $dir = $parent
    }
    throw "Not found: '$Target' starting from '$StartDir'"
}

function Get-GitRepoRoot {
    param([Parameter(Mandatory)][string]$StartDir)
    $gitPath = Find-Upwards -StartDir $StartDir -Target ".git"
    return (Split-Path -Parent $gitPath)
}

function Assert-Exists {
    param(
        [Parameter(Mandatory)][string]$Path,
        [Parameter(Mandatory)][string]$Label
    )
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Label not found: $Path"
    }
}

# =============================================================================
# A) Script -> RepoRoot -> Workspace
# =============================================================================
$ScriptPath     = $MyInvocation.MyCommand.Path
$ScriptDir      = Split-Path -Parent $ScriptPath
$ScriptRepoRoot = Get-GitRepoRoot -StartDir $ScriptDir
$WorkspaceDir   = Split-Path -Parent $ScriptRepoRoot

Write-Host "Script:     $ScriptPath"
Write-Host "ScriptDir:  $ScriptDir"
Write-Host "RepoRoot:   $ScriptRepoRoot"
Write-Host "Workspace:  $WorkspaceDir"

# =============================================================================
# B) Repos in workspace (adjust names here if needed)
# =============================================================================
$RadonUlzerRepo = Join-Path $WorkspaceDir "RadonUlzer"
$SSRRepo        = Join-Path $WorkspaceDir "SpaceShipRadar"
$DCSRepo        = Join-Path $WorkspaceDir "DroidControlShip"

Assert-Exists $RadonUlzerRepo "RadonUlzer repo"
Assert-Exists $SSRRepo        "SpaceShipRadar repo"
Assert-Exists $DCSRepo        "DroidControlShip repo"

Write-Host "`nRepos:"
Write-Host "  RadonUlzer:      $RadonUlzerRepo"
Write-Host "  SpaceShipRadar:  $SSRRepo"
Write-Host "  DroidControlShip:$DCSRepo"

# =============================================================================
# C) Derive targets (relative to workspace)
# =============================================================================
$RadonUlzer_RemoteControlSim = Join-Path $RadonUlzerRepo ".pio\build\RemoteControlSim\program.exe"
$DCS_LineFollowerSensorFusionSim = Join-Path $DCSRepo ".pio\build\LineFollowerSensorFusionSim\program.exe"

$SSR_Script            = Join-Path $SSRRepo "src\space_ship_radar\space_ship_radar.py"
$SSR_CalibrationFolder = Join-Path $SSRRepo "src\calibration\"

$RadonUlzer_Settings = Join-Path $RadonUlzerRepo "settings\settings.json"

# Path to webots-controller
if (-not $env:WEBOTS_HOME) { throw "WEBOTS_HOME is not set." }
$webotsController = Join-Path $env:WEBOTS_HOME "msys64\mingw64\bin\webots-controller.exe"

# Validate
Assert-Exists $RadonUlzer_RemoteControlSim "RemoteControlSim program.exe"
Assert-Exists $DCS_LineFollowerSensorFusionSim "DCS program.exe"
Assert-Exists $SSR_Script "SpaceShipRadar script"
Assert-Exists $SSR_CalibrationFolder "Calibration folder"
Assert-Exists $RadonUlzer_Settings "RadonUlzer settings.json"
Assert-Exists $webotsController "webots-controller.exe"

Write-Host "`nTargets:"
Write-Host "  RadonUlzer RemoteControlSim: $RadonUlzer_RemoteControlSim"
Write-Host "  DCS SensorFusionSim:         $DCS_LineFollowerSensorFusionSim"
Write-Host "  SSR script:                  $SSR_Script"
Write-Host "  SSR calibration folder:      $SSR_CalibrationFolder"
Write-Host "  RadonUlzer settings.json:    $RadonUlzer_Settings"
Write-Host "  webots-controller.exe:       $webotsController"

# =============================================================================
# IMPORTANT:
# This script assumes Webots is already running and the world contains <extern>
# controllers for the robots listed below.
# =============================================================================  

# =============================================================================
# Start RadonUlzer (Zumo / RemoteControlSim)
# =============================================================================
Start-Process -FilePath $webotsController `
    -ArgumentList @(
        "--robot-name=Zumo",
        "--stdout-redirect", $RadonUlzer_RemoteControlSim,
        "-c",
        "--supervisorRxCh", "1",
        "--supervisorTxCh", "2",
        "--serialRxCh",     "3",
        "--serialTxCh",     "4",
        "--settingsPath",   $RadonUlzer_Settings,
        "-v"
    )

# =============================================================================
# Start SpaceShipRadar (MyBot)
# =============================================================================
Start-Process -FilePath $webotsController `
    -ArgumentList @(
        "--robot-name=MyBot",
        "--stdout-redirect", $SSR_Script
    )

# =============================================================================
# Start DroidControlShip / LineFollowerSensorFusionSim
# =============================================================================
$DCSWorkDir = Split-Path -Parent $DCS_LineFollowerSensorFusionSim

Start-Process -FilePath $webotsController `
    -WorkingDirectory $DCSWorkDir `
    -ArgumentList @(
        "--robot-name=ZumoComSystem",
        "--stdout-redirect", $DCS_LineFollowerSensorFusionSim,
        "--cfgFilePath", "../../../data/config/config_zumo1.json",
        "--serialRxCh", "4",
        "--serialTxCh", "3",
        "-v"
    )
