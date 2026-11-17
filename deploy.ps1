# 一键部署到树莓派
# 用法: .\deploy.ps1 -Pi pi@192.168.4.193
param([string]$Pi = "pi@192.168.4.193")

$root = $PSScriptRoot
Write-Host "=== 检查连通性 ===" -ForegroundColor Cyan
ssh -o BatchMode=yes -o ConnectTimeout=15 $Pi "echo connected"
if ($LASTEXITCODE -ne 0) { throw "无法连接 $Pi" }

Write-Host "=== 上传运行时脚本 ===" -ForegroundColor Cyan
Get-ChildItem "$root\pi" -File | ForEach-Object {
    scp -o BatchMode=yes $_.FullName "${Pi}:/home/pi/"
    Write-Host ("  + " + $_.Name)
}

Write-Host "=== 上传配置 ===" -ForegroundColor Cyan
ssh -o BatchMode=yes $Pi "mkdir -p /home/pi/config"
Get-ChildItem "$root\config" -File | ForEach-Object {
    scp -o BatchMode=yes $_.FullName "${Pi}:/home/pi/config/"
    Write-Host ("  + config/" + $_.Name)
}

Write-Host "=== 上传底盘桥接 ===" -ForegroundColor Cyan
scp -o BatchMode=yes "$root\bridge\bridge_node.py" `
    "${Pi}:/home/pi/wheeltec_ws/src/wheeltec_bridge/wheeltec_bridge/"

Write-Host "=== 树莓派端收尾 ===" -ForegroundColor Cyan
ssh -o BatchMode=yes $Pi @"
cd /home/pi
chmod +x *.sh 2>/dev/null
python3 -m py_compile map_console.py depth_scan_fixed.py explorer.py && echo '编译通过'
cd /home/pi/wheeltec_ws && colcon build --packages-select wheeltec_bridge 2>&1 | tail -3
"@

Write-Host "`n完成。下一步：" -ForegroundColor Green
Write-Host "  ssh $Pi 'bash /home/pi/start_console.sh'"
Write-Host "  浏览器 http://<IP>:8080/"
