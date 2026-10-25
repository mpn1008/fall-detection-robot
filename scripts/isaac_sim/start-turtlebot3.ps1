param(
    [string]$IsaacPath = 'C:\isaac-sim',
    [string]$DdsProfile = '',
    [ValidateRange(0, 100)][int]$DomainId = 0,
    [string]$RobotUsd = '',
    [switch]$Headless
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$python = Join-Path $IsaacPath 'python.bat'
if (!(Test-Path -LiteralPath $python)) {
    throw "Isaac Sim python.bat not found at $python. Set -IsaacPath to the installation folder."
}
if ($env:AMENT_PREFIX_PATH -or $env:COLCON_PREFIX_PATH) {
    throw 'Use a fresh PowerShell without an external ROS installation sourced.'
}

# New releases moved bundled ROS libraries from ros2.bridge to ros2.core.
$candidates = @(
    (Join-Path $IsaacPath 'exts\isaacsim.ros2.core\jazzy\lib'),
    (Join-Path $IsaacPath 'exts\isaacsim.ros2.bridge\jazzy\lib')
)
$rosLib = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (!$rosLib) {
    throw 'Bundled Jazzy libraries not found. Check your Isaac Sim version; do not substitute Humble libraries silently.'
}
if (!$DdsProfile) {
    $DdsProfile = Join-Path $repo '.devcontainer\generated\windows-fastdds.xml'
}
$DdsProfile = (Resolve-Path -LiteralPath $DdsProfile).Path

$env:ROS_DISTRO = 'jazzy'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID = [string]$DomainId
$env:FASTRTPS_DEFAULT_PROFILES_FILE = $DdsProfile
$env:FASTDDS_DEFAULT_PROFILES_FILE = $DdsProfile
if (($env:PATH -split ';') -notcontains $rosLib) {
    $env:PATH = "$rosLib;$env:PATH"
}
$sceneArgs = @('-u', (Join-Path $PSScriptRoot 'turtlebot3_scene.py'))
if ($RobotUsd) { $sceneArgs += @('--robot-usd', $RobotUsd) }
if ($Headless) { $sceneArgs += '--headless' }
Write-Host "Starting TurtleBot3 test with ROS domain $DomainId and DDS profile $DdsProfile"
$versionFile = Join-Path $IsaacPath 'VERSION'
if (Test-Path -LiteralPath $versionFile) {
    Write-Host "Isaac Sim version: $((Get-Content -LiteralPath $versionFile -Raw).Trim())"
}
& $python @sceneArgs
$sceneExitCode = $LASTEXITCODE
if ($sceneExitCode -ne 0) {
    Write-Warning "TurtleBot3 scene exited with code $sceneExitCode. Check the Python traceback above."
}
exit $sceneExitCode
