# PowerShell translation of make_deps.sh

param (
    [string]$SdkPath,
    [string]$BuildType = "RelWithDebInfo"
)

if (-not $SdkPath) {
    Write-Host "Usage: $PSScriptRoot\<script-name>.ps1 <path-to-sdk>"
    exit 1
}

$env:SDK_PATH = (Resolve-Path $SdkPath).Path
Write-Host "SDK path: $($env:SDK_PATH)"

if (-not (Test-Path $env:SDK_PATH)) {
    Write-Host "WARN: SDK path does not exist"
    exit 1
}

$env:BUILD_TYPE = $BuildType
if (-z $BuildType) {
    Write-Host "Warning: BUILD_TYPE not set, using default: $($env:BUILD_TYPE)"
}

Set-Location "$env:SDK_PATH\.."
Write-Host "==> Cloning axdeps repository"
if (Test-Path "axdeps") {
    Write-Host "Info: axdeps directory already exists"
} else {
    git clone https://github.com/Adversarr/axdeps.git
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error: axdeps clone failed"
        exit 1
    }
}

Write-Host "==> Building axdeps"
$env:SDK_PATH = $env:SDK_PATH; $env:BUILD_TYPE = $env:BUILD_TYPE
& ".\axdeps\build.sh"
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: axdeps build failed"
    exit 1
}

Write-Host "==> Build and Install axdeps completed."
Write-Host "==> Removing axdeps directory"
$UserInput = Read-Host "Do you want to remove axdeps directory? (y/n): (default: n)"
if ($UserInput -eq 'y' -or $UserInput -eq 'Y') {
    Remove-Item -Recurse -Force "axdeps/build"
    Write-Host "Info: axdeps directory removed"
}

Write-Host "Success: axdeps build completed, Remember to set AX_SDK_PATH to $SdkPath when running cmake."
