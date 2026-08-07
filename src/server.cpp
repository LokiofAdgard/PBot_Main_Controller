#include "server.h"

AsyncWebServer server(80);

extern MController mController;

void server_init() {
    WiFi.softAP("PBOT_WIFI", "12345678");
    server.begin();

    // Main control page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">

        <style>
        *{
            margin:0;
            padding:0;
            box-sizing:border-box;
            user-select:none;
            -webkit-user-select:none;
        }

        body{
            background:#1b1b1b;
            color:white;
            font-family:Arial,Helvetica,sans-serif;
            display:flex;
            flex-direction:column;
            justify-content:center;
            align-items:center;
            min-height:100vh;
        }

        h2{
            margin-bottom:30px;
            font-size:30px;
            font-weight:600;
        }

        .grid{
            display:grid;
            grid-template-columns:repeat(3,90px);
            grid-template-rows:repeat(3,90px);
            gap:15px;
        }

        .btn{
            border:none;
            border-radius:18px;
            background:#2d7dff;
            color:white;
            font-size:42px;
            cursor:pointer;
            display:flex;
            justify-content:center;
            align-items:center;
            box-shadow:0 5px 12px rgba(0,0,0,.35);
            transition:transform .08s ease, background .15s ease;
            touch-action:manipulation;
        }

        .btn:active{
            background:#1257c9;
            transform:scale(0.94);
        }

        .stop{
            background:#e53935;
        }

        .stop:active{
            background:#b71c1c;
        }
        </style>

        </head>
        <body>

        <h2>PBOT Remote</h2>

        <div class="grid">
            <button class="btn" onmousedown="start('front_left')" onmouseup="stop()" onmouseleave="stop()">&#8598;</button>
            <button class="btn" onmousedown="start('front')"      onmouseup="stop()" onmouseleave="stop()">&#8593;</button>
            <button class="btn" onmousedown="start('front_right')"onmouseup="stop()" onmouseleave="stop()">&#8599;</button>

            <button class="btn" onmousedown="start('left')"  onmouseup="stop()" onmouseleave="stop()">&#8592;</button>
            <button class="btn stop" onclick="send('stop')">&#9632;</button>
            <button class="btn" onmousedown="start('right')" onmouseup="stop()" onmouseleave="stop()">&#8594;</button>

            <button class="btn" onmousedown="start('back_left')"  onmouseup="stop()" onmouseleave="stop()">&#8601;</button>
            <button class="btn" onmousedown="start('back')"       onmouseup="stop()" onmouseleave="stop()">&#8595;</button>
            <button class="btn" onmousedown="start('back_right')" onmouseup="stop()" onmouseleave="stop()">&#8600;</button>
        </div>

        <script>
        let interval = null;

        function start(cmd){
            send(cmd);  // immediate
            interval = setInterval(() => send(cmd), 120); // repeat
        }

        function stop(){
            clearInterval(interval);
            interval = null;
        }

        function send(cmd){
            fetch("/cmd?do=" + cmd);
        }
        </script>

        </body>
        </html>
        )rawliteral");
    });

    // Command handler
    server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (request->hasParam("do")) {
            String cmd = request->getParam("do")->value();

            if (cmd == "front") {
                mController.cmd_vel.m_left  = SPEED;
                mController.cmd_vel.m_right = SPEED;
            }
            if (cmd == "back") {
                mController.cmd_vel.m_left  = -SPEED;
                mController.cmd_vel.m_right = -SPEED;
            }
            if (cmd == "left") {
                mController.cmd_vel.m_left  = -SPEED;
                mController.cmd_vel.m_right = SPEED;
            }
            if (cmd == "right") {
                mController.cmd_vel.m_left  = SPEED;
                mController.cmd_vel.m_right = -SPEED;
            }
            if (cmd == "front_left") {
                mController.cmd_vel.m_left  = SPEED_LOW;
                mController.cmd_vel.m_right = SPEED;
            }
            if (cmd == "front_right") {
                mController.cmd_vel.m_left  = SPEED;
                mController.cmd_vel.m_right = SPEED_LOW;
            }
            if (cmd == "back_left") {
                mController.cmd_vel.m_left  = -SPEED_LOW;
                mController.cmd_vel.m_right = -SPEED;
            }
            if (cmd == "back_right") {
                mController.cmd_vel.m_left  = -SPEED;
                mController.cmd_vel.m_right = -SPEED_LOW;
            }
            if (cmd == "stop") {
                mController.cmd_vel.m_left  = 0;
                mController.cmd_vel.m_right = 0;
            }

            // mController.cmd_vel.timeout_c = 0;

            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing parameter");
        }
    });
}