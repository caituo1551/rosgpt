#!/usr/bin/env python3
# 這是 rosgpt 套件的一部分，設計用於 ROS1。

import os
import json
import openai
import rospy
from std_msgs.msg import String
from flask_restful import Resource, Api
from flask_cors import CORS
import threading
from flask import Flask, request, send_from_directory, jsonify
import socket
from datetime import datetime
import time

# ================= 初始化 =================
tasks_queue = []             # 全域的任務陣列
tasks_lock = threading.Lock() # 確保多執行緒存取安全

app = Flask(__name__)
CORS(app)
api = Api(app)

openai_api_key = os.getenv('OPENAI_API_KEY')
openai.api_key = openai_api_key

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WEBAPP_DIR = os.path.join(BASE_DIR, 'webapp')
STATIC_DIR = BASE_DIR

# ================= 工具函式 =================
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip_addr = s.getsockname()[0]
        s.close()
    except Exception:
        ip_addr = '127.0.0.1'
    return ip_addr

# ================= ROSGPT 節點 =================
class ROSGPTNode:
    def __init__(self):
        self.publisher = rospy.Publisher('voice_cmd', String, queue_size=10)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        rospy.loginfo(f"\U0001f4e2 訊息已發佈: {msg.data}")
def process_and_publish_chatgpt_response(chatgpt_ros_node, commands):
    """逐一發佈來自 ChatGPT 的指令"""
    for command in commands:
        chatgpt_ros_node.publish_message(json.dumps(command))  # 將指令轉為 JSON 格式發佈
        rospy.sleep(0.1)  # 防止指令發佈過快
# ================= Flask REST API 控制器 =================
class ROSGPTProxy(Resource):
    def __init__(self, chatgpt_ros_node):
        self.chatgpt_ros_node = chatgpt_ros_node

    def askGPT(self, text_command):
        prompt = '''
        請只回傳純粹的 JSON 陣列，不要包含自然語言描述。每筆指令必須包含 "action" 和 "params" 欄位，格式如下：
        [
            {"action": "move", "params": {"linear_speed": 1.0, "distance": 3.0, "is_forward": true}},
            {"action": "rotate", "params": {"angular_velocity": 2.0, "angle": 90, "is_clockwise": true}},
            {"action": "goto", "params": {"x": 2.0, "y": 1.5, "yaw": 1.57}}
        ]
        '''
        prompt += f"\n使用者指令: {text_command}"

        messages = [
            {"role": "system", "content": "你是負責生成有效機器指令的助理。請使用 JSON 陣列格式回應，內容不得包含文字描述。"},
            {"role": "user", "content": prompt}
        ]

        try:
            response = openai.ChatCompletion.create(model="gpt-4", messages=messages)
        except Exception as e:
            rospy.logerr(f"❌ GPT 呼叫錯誤: {e}")
            return None

        chatgpt_response = response.choices[0].message['content'].strip()
        try:
            if not chatgpt_response.startswith("["):
                chatgpt_response = "[" + chatgpt_response + "]"
            commands = json.loads(chatgpt_response)
            if isinstance(commands, dict):
                commands = [commands]
            return commands
        except json.JSONDecodeError:
            rospy.logerr("❌ 無法解析 GPT 回應為 JSON 格式")
            rospy.loginfo(f"GPT 回應（原始）: {chatgpt_response}")
            return None

    def post(self):
        text_command = request.form['text_command']
        rospy.loginfo(f"[ROSGPT] 收到的指令: {text_command}")
        now_str = datetime.now().strftime("%H:%M:%S")
        commands = self.askGPT(text_command)
        if not commands:
            with tasks_lock:
                tasks_queue.append({
                    "original_command": text_command,
                    "subcommands": [],
                    "status": "error",
                    "timestamp": now_str,
                    "error_message": "GPT 回傳格式無法解析"
                })
            return {'error': '無法從 GPT 回應中生成有效的 JSON 指令。'}

        with tasks_lock:
            tasks_queue.append({
                "original_command": text_command,
                "subcommands": commands,
                "status": "pending",
                "timestamp": now_str
            })
        return jsonify({"response": commands})

# ================= 指令執行緒 =================
def worker_thread(chatgpt_ros_node):
    while not rospy.is_shutdown():
        with tasks_lock:
            index_to_run = next((i for i, t in enumerate(tasks_queue) if t['status'] == 'pending'), None)

        if index_to_run is not None:
            with tasks_lock:
                tasks_queue[index_to_run]['status'] = 'running'
            subcommands = tasks_queue[index_to_run]['subcommands']
            for cmd in subcommands:
                chatgpt_ros_node.publish_message(json.dumps(cmd))
                rospy.sleep(1)
            with tasks_lock:
                tasks_queue[index_to_run]['status'] = 'done'

        rospy.sleep(1)

# ================= 靜態檔案路由 =================
@app.route('/tasks', methods=['GET'])
def get_tasks():
    with tasks_lock:
        return jsonify({"tasks": tasks_queue})

@app.route('/')
def index():
    return send_from_directory(WEBAPP_DIR, 'index.html')

@app.route('/script.js')
def serve_script():
    return send_from_directory(STATIC_DIR, 'script.js')

@app.route('/style.css')
def serve_style():
    return send_from_directory(STATIC_DIR, 'style.css')

# ================= 主程序 =================
def main():
    rospy.init_node('chatgpt_ros_node_' + str(int(time.time())), anonymous=True)
    rospy.loginfo("✅ 啟動 ROSGPT 節點 (ROS 1)")
    rospy.loginfo(f"[DEBUG] WEBAPP_DIR = {WEBAPP_DIR}")
    rospy.loginfo(f"[DEBUG] STATIC_DIR = {STATIC_DIR}")

    chatgpt_ros_node = ROSGPTNode()
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros_node,))

    worker = threading.Thread(target=worker_thread, args=(chatgpt_ros_node,), daemon=True)
    worker.start()

    ip_addr = get_local_ip()
    rospy.loginfo(f"\U0001f4f0 Flask 前端網址：http://{ip_addr}:5000")

    try:
        app.run(debug=False, host='0.0.0.0', port=5000)
    except Exception as e:
        rospy.logerr(f"❌ Flask 啟動失敗: {e}")

if __name__ == '__main__':
    if not rospy.core.is_initialized():
        main()
