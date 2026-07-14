#!/usr/bin/env python3
import json
import socket
import struct
import time
import threading
from dataclasses import dataclass, asdict

import rospy
from std_srvs.srv import Trigger, TriggerResponse


@dataclass
class PortState:
    connected: bool = False
    last_frame_time: float = 0.0
    frame_count: int = 0
    last_msg_type: int = -1
    last_msg_id: int = -1
    last_error: str = ""
    extra: dict = None

    def healthy(self, max_age):
        return self.connected and (time.time() - self.last_frame_time) <= max_age


class AiuiPortProbe(threading.Thread):
    def __init__(self, name, ip, port, header_mode, timeout=3.0):
        super().__init__(daemon=True)
        self.name = name
        self.ip = ip
        self.port = port
        self.header_mode = header_mode  # "semantic" = 7 bytes, "media" = 9 bytes
        self.timeout = timeout
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.state = PortState(extra={})

    def snapshot(self):
        with self.lock:
            return asdict(self.state)

    def set_error(self, text):
        with self.lock:
            self.state.connected = False
            self.state.last_error = text

    def recv_exact(self, sock, n):
        data = bytearray()
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def parse_header(self, sock):
        if self.header_mode == "semantic":
            header = self.recv_exact(sock, 7)
            if not header:
                return None
            sync, user_id, msg_type, msg_len, msg_id = struct.unpack("<BBBHH", header)
            return sync, user_id, msg_type, msg_len, msg_id

        header = self.recv_exact(sock, 9)
        if not header:
            return None
        sync, user_id, msg_type, msg_len, msg_id = struct.unpack("<BBB I H", header)
        return sync, user_id, msg_type, msg_len, msg_id

    def handle_payload(self, msg_type, msg_id, payload):
        extra = {}

        if self.name == "audio":
            # 音频协议：0x0a = VAD帧, 0x0c = 连续音频帧
            if msg_type in (0x0A, 0x0C) and len(payload) >= 8:
                vad = payload[0]
                channel = payload[1]
                frame_no = struct.unpack("<I", payload[4:8])[0]
                extra = {
                    "vad": vad,
                    "channel": channel,
                    "frame_no": frame_no,
                    "pcm_bytes": max(0, len(payload) - 8),
                }

        elif self.name == "video":
            # 视频端口会混合图像格式、图像帧、人脸检测 JSON。
            # 这里不强依赖具体 msg_type，能稳定收到合法帧即可判定视频链路活着。
            try:
                text = payload.decode("utf-8")
                obj = json.loads(text)
                extra = {"json": obj}
            except Exception:
                extra = {"binary_bytes": len(payload)}

        elif self.name == "semantic":
            try:
                text = payload.decode("utf-8")
                extra = {"json": json.loads(text)}
            except Exception:
                extra = {"bytes": len(payload)}

        with self.lock:
            self.state.connected = True
            self.state.last_frame_time = time.time()
            self.state.frame_count += 1
            self.state.last_msg_type = msg_type
            self.state.last_msg_id = msg_id
            self.state.last_error = ""
            self.state.extra = extra

    def run(self):
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            try:
                with socket.create_connection((self.ip, self.port), timeout=self.timeout) as sock:
                    sock.settimeout(self.timeout)

                    with self.lock:
                        self.state.connected = True
                        self.state.last_error = ""

                    while not rospy.is_shutdown() and not self.stop_event.is_set():
                        parsed = self.parse_header(sock)
                        if not parsed:
                            raise ConnectionError("socket closed")

                        sync, user_id, msg_type, msg_len, msg_id = parsed
                        if sync != 0xA5 or user_id != 0x01:
                            raise ValueError(f"bad header sync={sync:#x}, user={user_id:#x}")

                        payload_plus_check = self.recv_exact(sock, msg_len + 1)
                        if not payload_plus_check:
                            raise ConnectionError("payload closed")

                        payload = payload_plus_check[:msg_len]
                        self.handle_payload(msg_type, msg_id, payload)

            except Exception as e:
                self.set_error(str(e))
                time.sleep(1.0)


class AiuiDeviceHealthNode:
    def __init__(self):
        self.ip = rospy.get_param("~aiui_ip", "192.168.8.206")
        self.semantic_port = rospy.get_param("~semantic_port", 19199)
        self.video_port = rospy.get_param("~video_port")
        self.audio_port = rospy.get_param("~audio_port")
        self.max_age = rospy.get_param("~max_frame_age", 5.0)

        self.probes = [
            AiuiPortProbe("semantic", self.ip, self.semantic_port, "semantic"),
            AiuiPortProbe("video", self.ip, self.video_port, "media"),
            AiuiPortProbe("audio", self.ip, self.audio_port, "media"),
        ]

        for p in self.probes:
            p.start()

        self.service = rospy.Service("~check", Trigger, self.handle_check)

    def handle_check(self, _req):
        data = {}
        ok = True

        for p in self.probes:
            state = p.snapshot()
            state["healthy"] = p.state.healthy(self.max_age)
            state["age_sec"] = round(time.time() - state["last_frame_time"], 3) if state["last_frame_time"] else None
            data[p.name] = state
            ok = ok and state["healthy"]

        if not data["video"]["healthy"]:
            data["diagnosis"] = "视频端口无有效帧：优先检查摄像头接线、盒子视频服务、video_port配置。"
        elif not data["audio"]["healthy"]:
            data["diagnosis"] = "音频端口无有效PCM帧：优先检查麦克风链路、音频服务、audio_port配置。"
        elif not data["semantic"]["healthy"]:
            data["diagnosis"] = "19199语义端口异常：AIUI语义链路或网络连接异常。"
        else:
            data["diagnosis"] = "AIUI语义、视频、音频端口均有近期有效数据。"

        return TriggerResponse(
            success=ok,
            message=json.dumps(data, ensure_ascii=False)
        )


if __name__ == "__main__":
    rospy.init_node("aiui_device_health")
    AiuiDeviceHealthNode()
    rospy.spin()