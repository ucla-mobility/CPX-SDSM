# Validation runs: valid_n10 (10-veh) and valid_a200 (alpha=2.0, sparse 150-veh).
# Periodic, 150 s, seed 0. Detached-safe.
# Toolchain locations: override via env vars OMNETPP_ROOT / VEINS_ROOT / SUMO_ROOT
# (defaults below match the original Windows setup this study was run on).
param([int]$Limit = 150)
$ErrorActionPreference = "Continue"
$OMNET = if ($env:OMNETPP_ROOT) { $env:OMNETPP_ROOT } else { "D:\sim\omnetpp-5.6.2" }; $VEINS = if ($env:VEINS_ROOT) { $env:VEINS_ROOT } else { "D:\sim\veins-veins-5.2" }
$SUMO = if ($env:SUMO_ROOT) { $env:SUMO_ROOT } else { "D:\sim\sumo-1.8.0" }
$env:PATH = "$OMNET\bin;$VEINS\out\clang-release\src;$OMNET\tools\win64\mingw64\bin;$SUMO\bin;" + $env:PATH
$exe = "$PSScriptRoot\src\veins_ros_v2v_ucla.exe"; $sim = "$PSScriptRoot\simulations"
$ned = ".;../src;./apps;./networks;$VEINS/src/veins"
$status = "$sim\valid_status.log"
Set-Location $sim
"=== VALID START $(Get-Date -Format 'u') (${Limit}s) ===" | Set-Content $status
foreach ($cfg in @("valid_n10","valid_a200")) {
  Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
  Start-Sleep -Seconds 2
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg start" | Add-Content $status
  $sw = [Diagnostics.Stopwatch]::StartNew()
  & $exe omnetpp_valid.ini -c $cfg -r 0 -u Cmdenv -n $ned "--sim-time-limit=${Limit}s" *> "$sim\run_$cfg.log"
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg done exit=$LASTEXITCODE wall=$([int]$sw.Elapsed.TotalSeconds)s" | Add-Content $status
}
Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
"$(Get-Date -Format 'HH:mm:ss')  VALID DONE" | Add-Content $status
