# Run the three canonical configs sequentially (400 veh / 300s / seed 0),
# detached so closing any window can't kill them. They share SUMO port 9998,
# so they MUST run one at a time.
# Toolchain locations: override via env vars OMNETPP_ROOT / VEINS_ROOT / SUMO_ROOT
# (defaults below match the original Windows setup this study was run on).
$ErrorActionPreference = "Continue"
$OMNET = if ($env:OMNETPP_ROOT) { $env:OMNETPP_ROOT } else { "D:\sim\omnetpp-5.6.2" }
$VEINS = if ($env:VEINS_ROOT) { $env:VEINS_ROOT } else { "D:\sim\veins-veins-5.2" }
$SUMO = if ($env:SUMO_ROOT) { $env:SUMO_ROOT } else { "D:\sim\sumo-1.8.0" }
$env:PATH = "$OMNET\bin;$VEINS\out\clang-release\src;$OMNET\tools\win64\mingw64\bin;$SUMO\bin;" + $env:PATH
$exe = "$PSScriptRoot\src\veins_ros_v2v_ucla.exe"
$sim = "$PSScriptRoot\simulations"
$ned = ".;../src;./apps;./networks;$VEINS/src/veins"
$status = "$sim\run_all3_status.log"
Set-Location $sim
"=== START $(Get-Date -Format 'u') ===" | Set-Content $status
foreach ($cfg in @("Periodic","Greedy_v2","HybridSDSM_v2")) {
  Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
  Start-Sleep -Seconds 2
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg start" | Add-Content $status
  $sw = [Diagnostics.Stopwatch]::StartNew()
  & $exe omnetpp.ini -c $cfg -r 0 -u Cmdenv -n $ned "--sim-time-limit=300s" *> "$sim\run_$cfg.log"
  $code = $LASTEXITCODE
  $sw.Stop()
  "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg done exit=$code wall=$([int]$sw.Elapsed.TotalSeconds)s" | Add-Content $status
}
Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
"$(Get-Date -Format 'HH:mm:ss')  ALL SIMS DONE" | Add-Content $status
