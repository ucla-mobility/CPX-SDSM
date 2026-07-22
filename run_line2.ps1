# Three denser line scenarios (63, 100.5, 125.5 veh/km). Periodic, 30 s. VISIBLE.
# Toolchain locations: override via env vars OMNETPP_ROOT / VEINS_ROOT / SUMO_ROOT
# (defaults below match the original Windows setup this study was run on).
param([int]$Limit = 30)
$ErrorActionPreference = "Continue"
$OMNET = if ($env:OMNETPP_ROOT) { $env:OMNETPP_ROOT } else { "D:\sim\omnetpp-5.6.2" }; $VEINS = if ($env:VEINS_ROOT) { $env:VEINS_ROOT } else { "D:\sim\veins-veins-5.2" }
$SUMO = if ($env:SUMO_ROOT) { $env:SUMO_ROOT } else { "D:\sim\sumo-1.8.0" }
$env:PATH = "$OMNET\bin;$VEINS\out\clang-release\src;$OMNET\tools\win64\mingw64\bin;$SUMO\bin;" + $env:PATH
$exe = "$PSScriptRoot\src\veins_ros_v2v_ucla.exe"; $sim = "$PSScriptRoot\simulations"
$ned = ".;../src;./apps;./networks;$VEINS/src/veins"
$status = "$sim\line2_status.log"
Set-Location $sim
"=== LINE2 START $(Get-Date -Format 'u') (${Limit}s) ===" | Set-Content $status
foreach ($cfg in @("line_s16","line_s10","line_s8")) {
  Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
  Start-Sleep -Seconds 2
  Write-Host "`n========== RUNNING $cfg (limit ${Limit}s) ==========" -ForegroundColor Cyan
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg start" | Add-Content $status
  & $exe omnetpp_line.ini -c $cfg -r 0 -u Cmdenv -n $ned "--sim-time-limit=${Limit}s" 2>&1 | Tee-Object -FilePath "$sim\run_$cfg.log"
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg done exit=$LASTEXITCODE" | Add-Content $status
}
Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
"$(Get-Date -Format 'HH:mm:ss')  LINE2 DONE" | Add-Content $status
Write-Host "`nDONE. You can close this window." -ForegroundColor Green
