# Run after Windows restart. Imports only if SciSim-Ubuntu is not already registered.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$distro = 'SciSim-Ubuntu'
$image = Join-Path $taskRoot 'ubuntu-26.04.1-wsl-amd64.wsl'
$location = Join-Path $taskRoot '.wsl\Ubuntu'
if (-not (Test-Path -LiteralPath $image)) { throw "Missing Ubuntu image: $image" }
$registered = ((& wsl.exe --list --quiet 2>$null) -replace "`0", '')
if ($registered -notcontains $distro) {
    New-Item -ItemType Directory -Force -Path $location | Out-Null
    & wsl.exe --import $distro $location $image --version 2
    if ($LASTEXITCODE -ne 0) { throw 'Ubuntu import failed. Ensure Windows has restarted after enabling WSL.' }
}
$portableRoot = $taskRoot.Replace('\', '/')
$linuxRoot = (& wsl.exe --distribution $distro --user root --exec wslpath -a $portableRoot).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Could not start Ubuntu.' }
& wsl.exe --distribution $distro --user root --exec bash "$linuxRoot/scripts/bootstrap_wsl.sh" $linuxRoot
if ($LASTEXITCODE -ne 0) { throw 'Linux dependencies failed to install; inspect the preceding error.' }
Write-Host 'Ubuntu and dependencies ready. Run scene validation separately before claiming success.'
