import openai
import os
import json

openai.api_key = os.getenv('OPENAI_API_KEY')

def extract_json(response_text):
    """從 GPT 回應中提取 JSON 部分"""
    start = response_text.find('[')
    end = response_text.rfind(']') + 1
    if start != -1 and end != -1:
        return response_text[start:end]
    return None

def test_gpt_output(command):
    """測試 GPT 的 JSON 輸出"""
    prompt = '''
    請僅輸出有效的 JSON 指令陣列格式，不包含其他文字說明。每個指令必須包含 "action" 和 "params"。
    格式範例：
    [
        {{"action": "go_to_goal", "params": {{"location": {{"type": "str", "value": "bedroom"}}}}}},
        {{"action": "move", "params": {{"linear_speed": 0.5, "distance": 1.0, "is_forward": true}}}},
        {{"action": "rotate", "params": {{"angular_velocity": 10.0, "angle": 90, "is_clockwise": true}}}}
    ]
    現在的指令為: "{command}"
    '''
    # 調試輸出
    try:
        formatted_prompt = prompt.format(command=command)
        print(f"格式化後的 Prompt:\n{formatted_prompt}")
    except KeyError as e:
        print(f"格式化 Prompt 時出現錯誤: {e}")
        return

    messages = [
        {"role": "system", "content": "你是負責生成有效機器指令的助理。"},
        {"role": "user", "content": formatted_prompt}
    ]

    try:
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=messages,
        )
        chatgpt_response = response.choices[0].message['content'].strip()
        print(f"GPT 回應:\n{chatgpt_response}")

        # 提取 JSON 部分
        extracted_json = extract_json(chatgpt_response)
        if extracted_json:
            try:
                parsed_json = json.loads(extracted_json)
                print(f"成功解析的 JSON:\n{parsed_json}")
            except json.JSONDecodeError:
                print("提取的 JSON 部分解析失敗")
        else:
            print("無法提取 JSON 部分")
    except Exception as e:
        print(f"GPT 請求失敗: {e}")

if __name__ == "__main__":
    # 測試用例
    test_gpt_output("前進3公尺")
