# Power/MCS sweep: runs the six [Config sw_*] from omnetpp_sweep.ini sequentially
# (sparse 150-veh scenario, Periodic 10 Hz, 150 s, seed 0). No ini editing, no
# command-line param overrides -> no encoding/quoting pitfalls. Detached-safe.
# Toolchain locations: override via env vars OMNETPP_ROOT / VEINS_ROOT / SUMO_ROOT
# (defaults below match the original Windows setup this study was run on).
param([int]$Limit = 150)
$ErrorActionPreference = "Continue"
$OMNET = if ($env:OMNETPP_ROOT) { $env:OMNETPP_ROOT } else { "D:\sim\omnetpp-5.6.2" }
$VEINS = if ($env:VEINS_ROOT) { $env:VEINS_ROOT } else { "D:\sim\veins-veins-5.2" }
$SUMO = if ($env:SUMO_ROOT) { $env:SUMO_ROOT } else { "D:\sim\sumo-1.8.0" }
$env:PATH = "$OMNET\bin;$VEINS\out\clang-release\src;$OMNET\tools\win64\mingw64\bin;$SUMO\bin;" + $env:PATH
$exe = "$PSScriptRoot\src\veins_ros_v2v_ucla.exe"
$sim = "$PSScriptRoot\simulations"
$ned = ".;../src;./apps;./networks;$VEINS/src/veins"
$status = "$sim\sweep_status.log"
Set-Location $sim
"=== SWEEP START $(Get-Date -Format 'u') (sparse 150 veh, ${Limit}s) ===" | Set-Content $status
$configs = @("sw_p20_b6","sw_p23_b6","sw_p26_b6","sw_p30_b6","sw_p33_b6","sw_p20_b3")
foreach ($cfg in $configs) {
  Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
  Start-Sleep -Seconds 2
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg start" | Add-Content $status
  $sw = [Diagnostics.Stopwatch]::StartNew()
  & $exe omnetpp_sweep.ini -c $cfg -r 0 -u Cmdenv -n $ned "--sim-time-limit=${Limit}s" *> "$sim\run_$cfg.log"
  $code = $LASTEXITCODE; $sw.Stop()
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg done exit=$code wall=$([int]$sw.Elapsed.TotalSeconds)s" | Add-Content $status
}
Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
"$(Get-Date -Format 'HH:mm:ss')  SWEEP DONE" | Add-Content $status
