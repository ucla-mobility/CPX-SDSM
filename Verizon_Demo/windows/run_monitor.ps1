param(
    [string]$Python = "C:\pixi_ws\.pixi\envs\default\python.exe",
    [double]$Latitude = 34.067086,
    [double]$Longitude = -118.445280
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$DeviceFile = Join-Path $ProjectRoot "clients\infrastructure_gateway\registration.json"
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Python runtime not found: $Python"
}
if (-not (Test-Path -LiteralPath $DeviceFile)) {
    throw "Software registration not found: $DeviceFile"
}

$RunId = "pc_monitor_" + (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
$LogDir = Join-Path $ProjectRoot "logs\$RunId"
New-Item -ItemType Directory -Force -Path $LogDir,(Join-Path $ProjectRoot "run") | Out-Null
Set-Content -LiteralPath (Join-Path $ProjectRoot "run\pc_monitor_current.txt") `
    -Value $LogDir -Encoding ASCII

$env:PYTHONPATH = "$ProjectRoot\app;$ProjectRoot\vendor\python-etx-samples\src"
$env:ETX_FORCE_CODEC_LITE = "1"
Write-Host "PC relay logs: $LogDir"
Write-Host "Keep this window open; stop with Ctrl+C."
& $Python (Join-Path $ProjectRoot "app\etx_software_relay.py") `
    --device-file $DeviceFile `
    --lat $Latitude `
    --lon $Longitude `
    --infrastructure-sender-id "ucla-infrastructure-nw" `
    --vehicle-sender-id "ucla-vehicle-01" `
    --log-dir $LogDir
exit $LASTEXITCODE
