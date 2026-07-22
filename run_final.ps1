# FINAL summary batch: 6 groups x 20 seeds (UCLA map, alpha=2.0). Each group runs
# all 20 seeds in one invocation via -r 0..19. Detached-safe.
# Toolchain locations: override via env vars OMNETPP_ROOT / VEINS_ROOT / SUMO_ROOT
# (defaults below match the original Windows setup this study was run on).
param([int]$Limit = 60)
$ErrorActionPreference = "Continue"
$OMNET = if ($env:OMNETPP_ROOT) { $env:OMNETPP_ROOT } else { "D:\sim\omnetpp-5.6.2" }; $VEINS = if ($env:VEINS_ROOT) { $env:VEINS_ROOT } else { "D:\sim\veins-veins-5.2" }
$SUMO = if ($env:SUMO_ROOT) { $env:SUMO_ROOT } else { "D:\sim\sumo-1.8.0" }
$env:PATH = "$OMNET\bin;$VEINS\out\clang-release\src;$OMNET\tools\win64\mingw64\bin;$SUMO\bin;" + $env:PATH
$exe = "$PSScriptRoot\src\veins_ros_v2v_ucla.exe"; $sim = "$PSScriptRoot\simulations"
$ned = ".;../src;./apps;./networks;$VEINS/src/veins"
$status = "$sim\final_status.log"
Set-Location $sim
"=== FINAL START $(Get-Date -Format 'u') (${Limit}s, 20 seeds/group) ===" | Set-Content $status
foreach ($cfg in @("per_n10","hyb_n10","per_n150","hyb_n150","per_n400","hyb_n400")) {
  Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
  Start-Sleep -Seconds 2
  "$(Get-Date -Format 'HH:mm:ss')  GROUP $cfg (seeds 0..19) start" | Add-Content $status
  $sw = [Diagnostics.Stopwatch]::StartNew()
  & $exe omnetpp_final.ini -c $cfg -r 0..19 -u Cmdenv -n $ned "--sim-time-limit=${Limit}s" *> "$sim\run_final_$cfg.log"
  "$(Get-Date -Format 'HH:mm:ss')  GROUP $cfg done exit=$LASTEXITCODE wall=$([int]$sw.Elapsed.TotalSeconds)s" | Add-Content $status
}
Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
"$(Get-Date -Format 'HH:mm:ss')  FINAL DONE" | Add-Content $status
