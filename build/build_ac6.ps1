# build_ac6.ps1 -- 用 AC6（armclang）+ Keil MDK-Community 编出可烧录的 gimbal.hex（本机自包含流程）
#
#   pwsh -NoProfile -ExecutionPolicy Bypass -File fw\keil_ac6\build_ac6.ps1 `
#        -Tree 'D:\works\RMTEST\work\Infantry2026\2026_Embedding_Infantry-main\2026_Embedding_Infantry\gimbal' `
#        -UserCode 'D:\works\RM\mujoco_infantry\fw\src\2026-10-25_periodic_ff\user_code'
#
# 做了什么：
#   1. 从 -Tree 拷 Core/Drivers/Middlewares/MDK-ARM（Cube 生成部分，不含 uvprojx）
#   2. 用 -UserCode 覆盖 user_code，并打上 AC6 迁移补丁（apply_ac6_patches.py）
#   3. 用 fw/keil_ac6/gimbal.uvprojx（已迁移：uAC6=1 + FreeRTOS GCC 移植层）+ 移植层源文件
#   4. UV4 -b 命令行编译，产出 <OutDir>\MDK-ARM\gimbal\gimbal.hex
#
# 注意：Keil MDK 必须是**已激活 Community/正版授权**的（Lite 版 32 KB 链接上限会挡住本固件）。
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Tree,
    [Parameter(Mandatory = $true)][string]$UserCode,
    [string]$OutDir = 'D:\works\RM\temporal_resource\keil_ac6\build_tree',
    [string]$Uv4 = 'C:\Users\18777\AppData\Local\Keil_v5\UV4\UV4.exe',
    [string]$Python = 'C:\Users\18777\anaconda3\envs\Py3_11\python.exe',
    [switch]$Keep
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not (Test-Path $Uv4)) { throw "UV4.exe not found: $Uv4" }
if (-not (Test-Path $Tree)) { throw "Keil tree not found: $Tree" }
if (-not (Test-Path (Join-Path $UserCode 'module\gimbal.cpp'))) { throw "UserCode 里没有 module\gimbal.cpp: $UserCode" }

if (-not $Keep -and (Test-Path $OutDir)) { Remove-Item $OutDir -Recurse -Force }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

foreach ($sub in @('Core', 'Drivers', 'Middlewares')) {
    Copy-Item (Join-Path $Tree $sub) (Join-Path $OutDir $sub) -Recurse -Force
}
New-Item -ItemType Directory -Force -Path (Join-Path $OutDir 'MDK-ARM') | Out-Null
Copy-Item (Join-Path $Tree 'MDK-ARM\*') (Join-Path $OutDir 'MDK-ARM') -Force -ErrorAction SilentlyContinue
Copy-Item $UserCode (Join-Path $OutDir 'user_code') -Recurse -Force
Copy-Item (Join-Path $here 'gimbal.uvprojx') (Join-Path $OutDir 'MDK-ARM\gimbal.uvprojx') -Force
$portDst = Join-Path $OutDir 'Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F'
New-Item -ItemType Directory -Force -Path $portDst | Out-Null
Copy-Item (Join-Path $here 'FreeRTOS_GCC_ARM_CM4F\*') $portDst -Force

Write-Host '[patch] AC6 迁移补丁'
& $Python (Join-Path $here 'apply_ac6_patches.py') --user-code (Join-Path $OutDir 'user_code')
if ($LASTEXITCODE -ne 0) { throw 'apply_ac6_patches.py failed' }

$proj = Join-Path $OutDir 'MDK-ARM\gimbal.uvprojx'
$log = Join-Path $OutDir 'build_ac6.log'
Write-Host "[cc] uVision -b $proj"
$p = Start-Process -FilePath $Uv4 -ArgumentList @('-b', $proj, '-j0', '-o', $log) -Wait -PassThru -NoNewWindow
$tail = Get-Content $log -Encoding utf8
$tail | Select-String -Pattern 'error:|Error\(s\)|Program Size|FromELF' | ForEach-Object { Write-Host ('    ' + $_.Line) }
$hex = Join-Path $OutDir 'MDK-ARM\gimbal\gimbal.hex'
if (-not (Test-Path $hex)) { throw "构建失败（UV4 exit=$($p.ExitCode)），没生成 hex；日志：$log" }
$sha = (Get-FileHash $hex -Algorithm SHA256).Hash
Write-Host ''
Write-Host ('[ok] {0}  {1:N0} 字节  SHA256={2}' -f $hex, (Get-Item $hex).Length, $sha)
Write-Host '烧录前请按 fw/keil_ac6/README.md 的检查单做 SWD 冒烟，再归档+烧录。'
