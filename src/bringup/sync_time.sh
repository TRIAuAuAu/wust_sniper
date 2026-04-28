#!/bin/bash
# sync_time.sh - 安装 chrony 并同步系统时间（兼容双系统）
# 使用方法: sudo bash sync_time.sh

set -e

echo ">>> 检查 chrony 是否已安装..."
if ! command -v chronyc &> /dev/null; then
    echo ">>> 正在安装 chrony ..."
    apt-get update
    apt-get install -y chrony
else
    echo ">>> chrony 已安装"
fi

echo ">>> 备份原配置文件..."
cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.bak 2>/dev/null || true

echo ">>> 配置 chrony 使用公共 NTP 服务器..."
cat > /etc/chrony/chrony.conf << 'EOF'
# 使用公共 NTP 服务器池
pool 2.debian.pool.ntp.org iburst
pool ntp.ubuntu.com iburst

# 允许本地回环
bindcmdaddress 127.0.0.1

# 记录时钟漂移
driftfile /var/lib/chrony/chrony.drift

# 前3次同步若偏差超过1秒则直接步进
makestep 1.0 3

# 禁用硬件时钟同步，避免双系统（如 Windows）时间错乱
# rtcsync
EOF

# 将硬件时钟解读为本地时间，与 Windows 默认行为一致
echo ">>> 设置硬件时钟为本地时间..."
timedatectl set-local-rtc 1

echo ">>> 重启 chrony 服务..."
systemctl restart chrony

echo ">>> 强制立即同步... (waiting for sync)"
chronyc -a makestep 0.1 3
sleep 3

echo ""
echo ">>> 当前同步状态:"
chronyc tracking
chronyc sources -v
echo ""
echo "✓ 时间同步配置完成！"