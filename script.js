// script.js

// 全域變數
let recognition;
let outputDiv;


// 初始化 Web Speech API
function initialize() {
  recognition = new (window.SpeechRecognition || window.webkitSpeechRecognition)();
  recognition.interimResults = true;
  recognition.continuous = true;
  recognition.lang = 'zh-TW';

  recognition.onresult = function (event) {
    let interimTranscript = '';
    for (let i = event.resultIndex; i < event.results.length; ++i) {
      if (event.results[i].isFinal) {
        // 最終辨識結果
        outputDiv.innerHTML += '<p>' + event.results[i][0].transcript + '</p>';
      } else {
        // 中間過渡辨識結果
        interimTranscript += event.results[i][0].transcript;
      }
    }
    document.getElementById('interimOutput').innerText = interimTranscript;
  };

  recognition.onerror = function (event) {
    console.error('Recognition error:', event.error);
  };
}

// 開始錄音
function startRecording() {
  outputDiv = document.getElementById('output');
  outputDiv.innerHTML = '';
  recognition.start();
}

// 停止錄音
function stopRecording() {
  recognition.stop();
}

function speak(text) {
  // 
}


// 語音送指令函式
async function sendToRobot() {
  let command = document.getElementById('output').innerText;
  console.log('Voice Command to send:', command);

  try {
    let response = await fetch("/rosgpt", {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded"
      },
      body: new URLSearchParams({
        text_command: command
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    let jsonResponse = await response.json();
    console.log("Received response from robot:", jsonResponse);

    if (jsonResponse.error) {
      outputDiv.innerHTML += `<p>Error: ${jsonResponse.error}</p>`;
    } else if (jsonResponse.response) {
      outputDiv.innerHTML += `<p>Robot response: ${command}</p>`;
    } else {
      outputDiv.innerHTML += '<p>Unexpected server response</p>';
    }
  } catch (error) {
    console.error("Error sending command to robot:", error);
    outputDiv.innerHTML += '<p>Error: Failed to communicate with robot</p>';
  }
}

// ===== 新增文字輸入欄位之送指令函式 =====
async function sendTextCommandToRobot() {
  let textInput = document.getElementById('textCommand');
  let command = textInput.value.trim();
  if (!command) {
    console.warn("尚未輸入文字指令！");
    return;
  }

  console.log('文字指令:', command);
  try {
    let response = await fetch("/rosgpt", {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded"
      },
      body: new URLSearchParams({
        text_command: command
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    let jsonResponse = await response.json();
    console.log("Received response from robot (textCommand):", jsonResponse);

    if (jsonResponse.error) {
      outputDiv.innerHTML += `<p>Error: ${jsonResponse.error}</p>`;
    } else if (jsonResponse.response) {
      outputDiv.innerHTML += `<p>Robot response: ${JSON.stringify(jsonResponse)}</p>`;
    } else {
      outputDiv.innerHTML += `<p>Unexpected response: ${JSON.stringify(jsonResponse)}</p>`;
    }

  } catch (error) {
    console.error("Error sending text command to robot:", error);
    outputDiv.innerHTML += `<p>[Error] Failed to communicate with robot</p>`;
  }
}


function fetchSharedTasks() {
  fetch("/tasks")
    .then(res => res.json())
    .then(data => {
      renderTasksInOutput(data.tasks || []);
    })
    .catch(error => console.error("Error fetching tasks:", error));

}



function renderTasksInOutput(tasks) {
  let leftDiv = document.getElementById('leftPanel');
  let rightDiv = document.getElementById('rightPanel');
  leftDiv.innerHTML = "";
  rightDiv.innerHTML = "";

  if (!tasks || tasks.length === 0) {
    leftDiv.innerHTML = "<p>尚無任務</p>";
    return;
  }

  let leftHTML = "";
  // 只處理 pending 與 running 任務
  tasks.forEach(task => {
    if (task.status === "running") {
      leftHTML += `<p>[${task.timestamp}] ${task.original_command} (Running...)</p>`;
    } else if (task.status === "pending") {
      leftHTML += `<p>[${task.timestamp}] ${task.original_command} (Waiting)</p>`;
    }
  });
  leftDiv.innerHTML = leftHTML || "<p>無未完成任務</p>";

  // 右邊只顯示已完成的任務，並取最新7筆，由新到舊顯示
  let done = tasks.filter(t => t.status === "done");
  let rightHTML = "";
  if (done.length > 0) {
    let recentDone = done.slice(-7).reverse();
    recentDone.forEach(task => {
      rightHTML += `<p>[${task.timestamp}] ${task.original_command} (Complete)</p>`;
    });
  } else {
    rightHTML = "<p>無已完成任務</p>";
  }
  rightDiv.innerHTML = rightHTML;
}




// 初始化
window.onload = function () {
  outputDiv = document.getElementById('output');
  initialize();
  setInterval(fetchSharedTasks, 300);
};
