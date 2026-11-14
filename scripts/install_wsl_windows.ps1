# Run with Windows administrator privileges. Never reboots automatically.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$logDir = Join-Path $taskRoot 'outputs\setup'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
Start-Transcript -Path (Join-Path $logDir 'wsl-install.log') -Append
try {
    & winget.exe install --id Microsoft.WSL --exact --source winget --accept-package-agreements --accept-source-agreements --silent --disable-interactivity
    $packageCode = $LASTEXITCODE
    $features = @()
    foreach ($featureName in @('VirtualMachinePlatform', 'Microsoft-Windows-Subsystem-Linux')) {
        $result = Enable-WindowsOptionalFeature -Online -FeatureName $featureName -All -NoRestart
        $features += @{ Name = $featureName; RestartNeeded = $result.RestartNeeded }
    }
    @{ PackageExitCode = $packageCode; Features = $features; Completed = $true } |
        ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $logDir 'wsl-install-result.json')
} catch {
    @{ Completed = $false; Error = $_.Exception.Message } |
        ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $logDir 'wsl-install-result.json')
    throw
} finally {
    Stop-Transcript
}
