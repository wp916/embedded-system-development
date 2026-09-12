$ErrorActionPreference = 'Stop'

$skillRoot = Split-Path -Parent $PSScriptRoot
$validator = Join-Path $env:USERPROFILE '.codex\skills\.system\skill-creator\scripts\quick_validate.py'

if (-not (Test-Path -LiteralPath $validator)) {
    throw "Codex skill validator not found: $validator"
}

$env:PYTHONUTF8 = '1'
python $validator $skillRoot

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host "Validated skill source: $skillRoot"
