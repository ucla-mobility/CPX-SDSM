# Add seeds 1,2,3 to the four combo configs (seed 0 already done) for error bars.
# 4 configs x 3 seeds = 12 runs, full 400-veh scenario, 150 s. Detached-safe.
# -r <seed> drives seed-set=${runnumber} (fading + MAC backoff RNG); routes fixed.
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
$status = "$sim\combo_seeds_status.log"
Set-Location $sim
"=== COMBO-SEEDS START $(Get-Date -Format 'u') (400 veh, ${Limit}s, seeds 1-3) ===" | Set-Content $status
$configs = @("combo_per_p20","combo_hyb_p20","combo_per_p30","combo_hyb_p30")
foreach ($seed in 1,2,3) {
  foreach ($cfg in $configs) {
    Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2
    "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg seed=$seed start" | Add-Content $status
    $sw = [Diagnostics.Stopwatch]::StartNew()
    & $exe omnetpp_combo.ini -c $cfg -r $seed -u Cmdenv -n $ned "--sim-time-limit=${Limit}s" *> "$sim\run_${cfg}_s${seed}.log"
    $code = $LASTEXITCODE; $sw.Stop()
    "$(Get-Date -Format 'HH:mm:ss')  RUN $cfg seed=$seed done exit=$code wall=$([int]$sw.Elapsed.TotalSeconds)s" | Add-Content $status
  }
}
Get-Process sumo,sumo-gui,veins_ros_v2v_ucla -ErrorAction SilentlyContinue | Stop-Process -Force
"$(Get-Date -Format 'HH:mm:ss')  COMBO-SEEDS DONE" | Add-Content $status
