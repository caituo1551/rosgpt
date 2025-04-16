#!/usr/bin/env python3
# 用於向 ROSGPT 發送測試指令的簡單 HTTP 客戶端
import json
import requests

def send_text_command(text_command):
    """
    發送文字指令到 ROSGPT 系統並接收回應
    """
    data = {'text_command': text_command}
    try:
        response = requests.post('http://localhost:5000/rosgpt', data=data)
        if response.status_code == 200:
            response_str = response.content.decode('utf-8')
            try:
                response_dict = json.loads(response_str)
                print("\nGPT Response:", response_dict)
            except json.JSONDecodeError:
                print('[JSONDecodeError] 無效或空的 JSON 字串:', response_str)
        else:
            print(f"HTTP 錯誤 {response.status_code}: {response.content.decode('utf-8')}")
    except Exception as e:
        print(f"[Exception] 發送請求時出現錯誤: {str(e)}")

if __name__ == '__main__':
    while True:
        print("請輸入中文或英文指令 (例如: '前進 3 公尺，然後順時針旋轉 90 度'):")
        text_command = input("Enter a text command: ")
        send_text_command(text_command)
