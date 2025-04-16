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
import rospkg
from queue import Queue
from datetime import datetime
import time

tasks_queue = []             # 全域的任務陣列
tasks_lock = threading.Lock() # 確保多執行緒存取安全

# 初始化 Flask 應用程式並設定跨來源資源共享 (CORS) 策略
app = Flask(__name__)
CORS(app)
api = Api(app)

# 從環境變數中讀取 OpenAI API 金鑰
openai_api_key = os.getenv('OPENAI_API_KEY')
openai.api_key = openai_api_key

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WEBAPP_DIR = os.path.join(BASE_DIR, 'webapp')
STATIC_DIR = BASE_DIR  # script.js / style.css 就在專案根目錄




class ROSGPTNode:
    def __init__(self):
        
        self.publisher = rospy.Publisher('voice_cmd', String, queue_size=10)  # 宣告主題 voice_cmd

    def publish_message(self, message):
        """向主題 voice_cmd 發佈訊息"""
        msg = String()
        msg.data = message
        self.publisher.publish(msg)  # 發佈訊息
        rospy.loginfo(f"訊息已發佈: {msg.data}")  # 在日誌中記錄發佈的訊息

def process_and_publish_chatgpt_response(chatgpt_ros_node, commands):
    """逐一發佈來自 ChatGPT 的指令"""
    for command in commands:
        chatgpt_ros_node.publish_message(json.dumps(command))  # 將指令轉為 JSON 格式發佈
        rospy.sleep(0.1)  # 防止指令發佈過快

class ROSGPTProxy(Resource):
    def __init__(self, chatgpt_ros_node):
        self.chatgpt_ros_node = chatgpt_ros_node

    def askGPT(self, text_command):
        """
        使用 GPT 生成 JSON 格式的指令
        """
        # 使用 Few-shot Learning 設定提示內容
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
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=messages,
            )  # 向 GPT 發送請求
        except openai.error.InvalidRequestError as e:
            rospy.logerr(f"錯誤: {e}")  # 錯誤處理
            return None
        except Exception as e:
            rospy.logerr(f"意外錯誤: {e}")
            return None

        

        # 提取 GPT 的回應內容
        chatgpt_response = response.choices[0].message['content'].strip()
        try:
    # 若不是 JSON 陣列，就包裝為陣列
            if not chatgpt_response.startswith("["):
                chatgpt_response = "[" + chatgpt_response + "]"

            commands = json.loads(chatgpt_response)
            if isinstance(commands, dict):
                commands = [commands]
            return commands
        except json.JSONDecodeError:
            rospy.logerr("無法解析 GPT 回應為 JSON 格式。")
            rospy.loginfo(f"GPT 回應（原始內容）: {chatgpt_response}")  # 打印原始內容進行調試
            return None

    def post(self):
        """
        處理 HTTP POST 請求，生成並發佈 ROS 指令
        """
        text_command = request.form['text_command']  # 從請求中提取使用者指令
        rospy.loginfo(f"[ROSGPT] 收到的指令: {text_command}")
        now_str = datetime.now().strftime("%H:%M:%S")
        # 使用 GPT 生成機器指令
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


        # 將指令存入後端共享的 tasks_queue，並只設定狀態為 pending
        with tasks_lock:
            tasks_queue.append({
                "original_command": text_command,
                "subcommands": commands,   # 全部子指令
                "status": "pending",
                "timestamp": now_str
            })

        # 不再在此立即 publish 指令，讓 worker_thread 負責執行
        return jsonify({"response": commands})



def worker_thread(chatgpt_ros_node):
    while not rospy.is_shutdown():
        # 找到第一個 status=pending 任務
        with tasks_lock:
            index_to_run = None
            for i, task in enumerate(tasks_queue):
                if task["status"] == "pending":
                    index_to_run = i
                    break

        if index_to_run is not None:
            # 將該任務設為 running
            with tasks_lock:
                tasks_queue[index_to_run]["status"] = "running"

            # 執行 subcommands
            subcommands = tasks_queue[index_to_run]["subcommands"]
            for cmd in subcommands:
                chatgpt_ros_node.publish_message(json.dumps(cmd))
                rospy.sleep(1)  # 此處可根據需要使用實際完成的回饋機制

            # 執行完畢後更新狀態
            with tasks_lock:
                tasks_queue[index_to_run]["status"] = "done"

        rospy.sleep(1)






#提供一個 GET /tasks API，讓多個前端都能抓到相同的任務佇列
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



def main():
    rospy.init_node('chatgpt_ros_node_' + str(int(time.time())), anonymous=True)
    rospy.loginfo("啟動 ROSGPT 節點 (ROS 1)")
    rospy.loginfo(f"[DEBUG] WEBAPP_DIR = {WEBAPP_DIR}")
    rospy.loginfo(f"[DEBUG] STATIC_DIR = {STATIC_DIR}")

    chatgpt_ros_node = ROSGPTNode()  # 初始化 ROS 節點
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros_node,))  # 將路由添加到 API
    worker = threading.Thread(target=worker_thread, args=(chatgpt_ros_node,))
    worker.daemon = True
    worker.start()

    app.run(debug=False, host='0.0.0.0', port=5000)  # 啟動 Flask 應用程式

    

if __name__ == '__main__':
    if not rospy.core.is_initialized():
        main()

