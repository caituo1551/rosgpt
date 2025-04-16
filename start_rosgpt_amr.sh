#!/bin/bash

echo "=== 🧠 ROSGPT AMR 啟動腳本 ==="

# 自動抓取本機 IP
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo "✅ 偵測到本機 IP：$LOCAL_IP"

# 詢問 roscore 要開在哪裡
read -p "📍 請問 roscore 是否在本機上執行？[Y/n]: " LOCAL_CORE

if [[ "$LOCAL_CORE" == "n" || "$LOCAL_CORE" == "N" ]]; then
    read -p "🔧 請輸入 AMR 工業電腦 IP（roscore 執行主機）: " AMR_IP
    ROS_MASTER_URI="http://$AMR_IP:11311"
    ROS_HOSTNAME="$LOCAL_IP"
else
    ROS_MASTER_URI="http://$LOCAL_IP:11311"
    ROS_HOSTNAME="$LOCAL_IP"

    # 🔍 檢查是否已啟動 roscore，若否則自動啟動
    if ! pgrep -f rosmaster > /dev/null; then
        echo "🛠️ 尚未偵測到 roscore，正在背景啟動..."
        roscore > /dev/null 2>&1 &
        sleep 2
    else
        echo "✅ roscore 已在本機執行中"
    fi
fi

# 匯出環境變數至當前終端
export ROS_MASTER_URI=$ROS_MASTER_URI
export ROS_HOSTNAME=$ROS_HOSTNAME

# 顯示設定
echo
echo "📡 已套用 ROS 環境變數："
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"
echo "  ROS_HOSTNAME=$ROS_HOSTNAME"

# 寫入 ~/.bashrc（供未來所有終端使用）
sed -i '/ROS_MASTER_URI/d' ~/.bashrc
sed -i '/ROS_HOSTNAME/d' ~/.bashrc
echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc
echo "export ROS_HOSTNAME=$ROS_HOSTNAME" >> ~/.bashrc
echo "✅ 已將設定寫入 ~/.bashrc（新開終端將自動套用）"

# 啟動 roslaunch
echo
echo "🚀 啟動 roslaunch rosgpt_amr.launch ..."
sleep 1
roslaunch rosgpt rosgpt_amr.launch

