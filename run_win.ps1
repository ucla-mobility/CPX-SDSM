# Helper: run the ported CPX-SDSM sim on Windows (OMNeT++ 5.6.2 + Veins 5.2).
# Usage: powershell -File run_win.ps1 -Config Periodic -Seed 0 -Limit 30
# Toolchain locations: override via env vars OMNETPP_ROOT / VEINS_ROOT / SUMO_ROOT
# (defaults below match the original Windows setup this study was run on).
param(
  [string]$Config = "Periodic",
  [int]$Seed = 0,
  [int]$Limit = 30
)
$ErrorActionPreference = "Stop"
$OMNET = if ($env:OMNETPP_ROOT) { $env:OMNETPP_ROOT } else { "D:\sim\omnetpp-5.6.2" }
$VEINS = if ($env:VEINS_ROOT) { $env:VEINS_ROOT } else { "D:\sim\veins-veins-5.2" }
$SUMO = if ($env:SUMO_ROOT) { $env:SUMO_ROOT } else { "D:\sim\sumo-1.8.0" }
$env:PATH = "$OMNET\bin;$VEINS\out\clang-release\src;$OMNET\tools\win64\mingw64\bin;$SUMO\bin;" + $env:PATH
$exe = "$PSScriptRoot\src\veins_ros_v2v_ucla.exe"
$ned = ".;../src;./apps;./networks;$VEINS/src/veins"
Set-Location "$PSScriptRoot\simulations"
& $exe omnetpp.ini -c $Config -r $Seed -u Cmdenv -n $ned "--sim-time-limit=${Limit}s"
Write-Output ("EXITCODE: " + $LASTEXITCODE)
