param(
    [Parameter(Mandatory = $true)]
    [string]$MasterConfig,
    [string]$Python = "C:\pixi_ws\.pixi\envs\default\python.exe",
    [switch]$Force
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$TargetDir = Join-Path $ProjectRoot "clients\infrastructure_vehicle"
$Target = Join-Path $TargetDir "registration.json"
$ExistingVehicle = Join-Path $ProjectRoot "clients\vehicle\registration.json"
$RegisterScript = Join-Path $ProjectRoot "vendor\python-etx-samples\src\examples\0_registration_example.py"

if (-not (Test-Path -LiteralPath $MasterConfig)) {
    throw "Master config not found: $MasterConfig"
}
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Python runtime not found: $Python"
}
if ((Test-Path -LiteralPath $Target) -and -not $Force) {
    throw "Target already exists. Use -Force only when intentionally replacing it."
}

$Artifact = Get-Content -LiteralPath $MasterConfig -Raw | ConvertFrom-Json
$Identity = $Artifact.identity.attributes
$Summary = "$($Identity.clientType)/$($Identity.clientSubType)/$($Identity.vendorId)"
if ($Summary -ne "Vehicle/PassengerCar/UclaEvalDevice") {
    throw "Master config is not the required Vehicle identity."
}
# The upstream CLI reads this exact spelling from the JSON configuration.
$Artifact.logLevel = "INFO"
$TempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("etx_registration_" + [guid]::NewGuid())
New-Item -ItemType Directory -Force -Path $TempRoot | Out-Null
$TempConfig = Join-Path $TempRoot "vehicle_master.json"
$ConfigJson = $Artifact | ConvertTo-Json -Depth 30
$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($TempConfig, $ConfigJson, $Utf8NoBom)

try {
    $env:PYTHONPATH = "$ProjectRoot\vendor\python-etx-samples\src"
    $SavedErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $RegistrationOutput = & $Python $RegisterScript `
        --config $TempConfig --cert-dir $TempRoot 2>&1
    $RegistrationExitCode = $LASTEXITCODE
    $ErrorActionPreference = $SavedErrorActionPreference
    if ($RegistrationExitCode -ne 0) {
        $RegistrationOutput | Select-Object -Last 8 | Write-Error
        throw "Verizon Vehicle registration failed."
    }
    $Generated = Get-ChildItem -LiteralPath $TempRoot -Filter "*.json" -File |
        Where-Object { $_.FullName -ne $TempConfig } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -eq $Generated) {
        $RegistrationOutput | Select-Object -Last 8 | Write-Error
        throw "Registration succeeded but no registration artifact was found."
    }
    $New = Get-Content -LiteralPath $Generated.FullName -Raw | ConvertFrom-Json
    $Old = Get-Content -LiteralPath $ExistingVehicle -Raw | ConvertFrom-Json
    $NewIdentity = $New.frozen_config.identity.attributes
    $NewSummary = "$($NewIdentity.clientType)/$($NewIdentity.clientSubType)/$($NewIdentity.vendorId)"
    if ($NewSummary -ne "Vehicle/PassengerCar/UclaEvalDevice") {
        throw "Generated artifact has the wrong identity."
    }
    if ($New.registration.device_id -eq $Old.registration.device_id) {
        throw "Generated Vehicle Device ID is not distinct from the existing Vehicle."
    }
    New-Item -ItemType Directory -Force -Path $TargetDir | Out-Null
    Copy-Item -LiteralPath $Generated.FullName -Destination $Target -Force
    Write-Host "NEW_INFRASTRUCTURE_VEHICLE_REGISTERED"
    Write-Host "Artifact: $Target"
    Write-Host "Identity: $NewSummary"
    Write-Host "Expiry: $($New.registration.certificate.expiration_time)"
}
finally {
    if (Test-Path -LiteralPath $TempRoot) {
        Remove-Item -LiteralPath $TempRoot -Recurse -Force
    }
}
