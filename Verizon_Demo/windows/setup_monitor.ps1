param(
    [string]$Python = "C:\pixi_ws\.pixi\envs\default\python.exe"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Python runtime not found: $Python"
}

$env:PYTHONPATH = "$ProjectRoot\app;$ProjectRoot\vendor\python-etx-samples\src"
$env:ETX_FORCE_CODEC_LITE = "1"
& $Python -c "import google.protobuf, geohash, paho.mqtt.client, requests, socks"
if ($LASTEXITCODE -eq 0) {
    Write-Host "[setup] existing Python runtime dependencies: OK"
}
else {
    Write-Host "[setup] missing runtime dependency; installing from PyPI"
    # geohash is intentionally not installed from PyPI. The project bundles
    # app\geohash.py, avoiding python-geohash 0.9.x's Rust build requirement.
    & $Python -m pip install --disable-pip-version-check `
        "protobuf>=6.31.1,<7" "paho-mqtt>=2.1,<3" `
        "requests>=2.32,<3" "PySocks>=1.7,<2"
    if ($LASTEXITCODE -ne 0) {
        throw "Monitor dependency installation failed."
    }
}

& $Python -c "from etx_endpoint import EtxEndpoint; from etx_software_relay import SoftwareRelay; print('WINDOWS_MONITOR_IMPORTS_OK')"
if ($LASTEXITCODE -ne 0) {
    throw "Monitor import test failed."
}
Write-Host "WINDOWS_MONITOR_SETUP_OK"
