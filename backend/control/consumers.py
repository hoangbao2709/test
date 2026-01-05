import json
from channels.generic.websocket import AsyncWebsocketConsumer

# Dev: dữ liệu giả. Thực tế: đọc từ ROS và push qua channel layer.
FAKE_STATE = {
    "fps": 30, "battery": 85, "status": "Resting",
    "pose": {"x": 0, "y": 0, "yaw": 0.0},
}

class TelemetryConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        self.robot_id = self.scope['url_route']['kwargs']['robot_id']
        await self.accept()
        await self.send_json({"type": "hello", "robot": self.robot_id})

    async def receive(self, text_data=None, bytes_data=None):
        # Client có thể gửi "ping" => server trả snapshot telemetry
        try:
            msg = json.loads(text_data or "{}")
        except Exception:
            msg = {}

        if msg.get("type") == "ping":
            await self.send_json({"type": "telemetry", **FAKE_STATE})

    async def send_json(self, data):
        await self.send(text_data=json.dumps(data))
