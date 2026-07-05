# Expose the WSL2-hosted vizanti server on hal's Windows-side ZeroTier IP
# (hal.robodojo.net), so ZeroTier-connected phones/tablets can browse to
# http://hal.robodojo.net:5000 even though the container listens inside WSL.
#
# Run ONCE from an elevated (Administrator) PowerShell:
#   powershell -ExecutionPolicy Bypass -File tools\windows\vizanti-portproxy.ps1
#
# Background: this machine uses WSL2 NAT networking with a separate ZeroTier
# node inside WSL (halbuntu.robodojo.net -> vizanti works there directly).
# These rules additionally forward the Windows ZT address into WSL via the
# localhost relay (reboot-stable; no dependency on the dynamic WSL NAT IP).
# Port 5000 = web UI, 5001 = rosbridge websocket (the browser needs both).

$ztIp = "10.147.20.20"   # hal.robodojo.net on the robodojo ZeroTier network
$ports = 5000, 5001

foreach ($p in $ports) {
    netsh interface portproxy delete v4tov4 listenaddress=$ztIp listenport=$p 2>$null
    netsh interface portproxy add v4tov4 listenaddress=$ztIp listenport=$p connectaddress=127.0.0.1 connectport=$p
}

# Allow inbound on the ZeroTier adapter (Windows Firewall blocks by default)
foreach ($p in $ports) {
    $ruleName = "Vizanti WSL forward $p"
    if (-not (Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue)) {
        New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow `
            -Protocol TCP -LocalPort $p -LocalAddress $ztIp | Out-Null
    }
}

netsh interface portproxy show v4tov4
Write-Host "Done. Test from a ZeroTier device: http://hal.robodojo.net:5000"
