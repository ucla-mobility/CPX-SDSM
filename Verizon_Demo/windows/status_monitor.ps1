$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$CurrentFile = Join-Path $ProjectRoot "run\pc_monitor_current.txt"
if (-not (Test-Path -LiteralPath $CurrentFile)) {
    throw "No monitor run has been recorded."
}
$LogDir = (Get-Content -LiteralPath $CurrentFile -Raw).Trim()
$StatusFile = Join-Path $LogDir "status.json"
if (-not (Test-Path -LiteralPath $StatusFile)) {
    throw "Status not available yet: $StatusFile"
}
$Status = Get-Content -LiteralPath $StatusFile -Raw | ConvertFrom-Json
Write-Host "READY=true"
Write-Host "IDENTITY=$($Status.identity)"
Write-Host "INFRA_SESSION_LEARNED=$($Status.endpoint_session_ready.'ucla-infrastructure-nw')"
Write-Host "VEHICLE_SESSION_LEARNED=$($Status.endpoint_session_ready.'ucla-vehicle-01')"
Write-Host "BSM_DATA_RX=$($Status.stats.bsm_data_rx)"
Write-Host "TIM_PUBLISHED=$($Status.stats.tim_published)"
Write-Host "INFRA_TO_VEHICLE=$($Status.stats.infra_to_vehicle)"
Write-Host "VEHICLE_TO_INFRA=$($Status.stats.vehicle_to_infra)"
Write-Host "TIM_FAILED=$($Status.stats.tim_failed)"
Write-Host "UNKNOWN_SENDER=$($Status.stats.unknown_sender)"
Write-Host "LOG_DIR=$LogDir"
