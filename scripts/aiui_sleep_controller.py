#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket
import threading
import time

import rospy
from std_srvs.srv import Trigger, TriggerResponse


AIUI_HOST = "192.168.8.206"
AIUI_PORT = 19199
SOCKET_TIMEOUT = 2.0

SYNC_HEAD = 0xA5
USER_ID = 0x01

MSG_TYPE_HANDSHAKE = 0x01
MSG_TYPE_MASTER_CONTROL = 0x05
MSG_TYPE_ACK = 0xFF

CMD_RESET_WAKEUP = 8
CMD_TTS = 27
TTS_CANCEL = 2

_message_id_lock = threading.Lock()
_message_id = 0


def _next_message_id():
    global _message_id
    with _message_id_lock:
        _message_id = (_message_id + 1) & 0xFFFF
        if _message_id == 0:
            _message_id = 1
        return _message_id


def _uint16_le(value):
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def calc_checkcode(frame_without_checkcode):
    return ((~sum(frame_without_checkcode) + 1) & 0xFF)


def build_aiui_frame(message_type, payload=b"", message_id=None):
    if isinstance(payload, str):
        payload = payload.encode("utf-8")

    if message_id is None:
        message_id = _next_message_id()

    if len(payload) > 0xFFFF:
        raise ValueError("AIUI payload is too large for uint16 length")

    frame = bytes((SYNC_HEAD, USER_ID, message_type))
    frame += _uint16_le(len(payload))
    frame += _uint16_le(message_id)
    frame += payload
    return frame + bytes((calc_checkcode(frame),))


def build_handshake_frame(message_id=None):
    return build_aiui_frame(MSG_TYPE_HANDSHAKE, b"\xA5\x00\x00\x00", message_id)


def build_master_control_payload(content_type, content):
    payload = {
        "type": content_type,
        "content": content,
    }
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def build_reset_wakeup_frame():
    payload = build_master_control_payload(
        "aiui_msg",
        {
            "msg_type": CMD_RESET_WAKEUP,
            "arg1": 0,
            "arg2": 0,
            "params": "",
            "data": "",
        },
    )
    return build_aiui_frame(MSG_TYPE_MASTER_CONTROL, payload)


def build_voice_enable_frame(enable):
    """
    关闭/开启 AIUI 播放声音。

    enable=False 表示关闭声音播放。
    注意：这个命令不一定能打断已经缓存或已经开始播放的音频。
    """
    payload = build_master_control_payload(
        "voice",
        {
            "enable_voice": bool(enable),
        },
    )
    return build_aiui_frame(MSG_TYPE_MASTER_CONTROL, payload)


def build_tts_stop_frame():
    """
    停止盒子 TTS 播放。

    如果盒子固件支持 type=tts/action=stop，这条可能能打断 TTS。
    你当前看到 Broken pipe，说明固件可能收到这类消息后主动断开 socket。
    """
    payload = build_master_control_payload(
        "tts",
        {
            "action": "stop",
        },
    )
    return build_aiui_frame(MSG_TYPE_MASTER_CONTROL, payload)


def build_tts_cancel_frame():
    """
    取消 AIUI SDK TTS。

    CMD_TTS=27，arg1=2 表示 CANCEL。不同固件支持情况可能不同。
    """
    payload = build_master_control_payload(
        "aiui_msg",
        {
            "msg_type": CMD_TTS,
            "arg1": TTS_CANCEL,
            "arg2": 0,
            "params": "",
            "data": "",
        },
    )
    return build_aiui_frame(MSG_TYPE_MASTER_CONTROL, payload)


def parse_aiui_frame(data):
    if len(data) < 8:
        return None
    if data[0] != SYNC_HEAD or data[1] != USER_ID:
        return None

    payload_len = data[3] | (data[4] << 8)
    frame_len = 7 + payload_len + 1
    if len(data) < frame_len:
        return None

    frame = data[:frame_len]
    expected_checkcode = calc_checkcode(frame[:-1])
    if frame[-1] != expected_checkcode:
        rospy.logwarn(
            "[SleepCtrl] Invalid AIUI checkcode: got=0x%02X expected=0x%02X",
            frame[-1],
            expected_checkcode,
        )
        return None

    return {
        "type": frame[2],
        "length": payload_len,
        "message_id": frame[5] | (frame[6] << 8),
        "payload": frame[7:-1],
        "raw": frame,
    }


def _recv_one_frame(sock):
    try:
        data = sock.recv(4096)
    except socket.timeout:
        return None

    if not data:
        return None

    frame = parse_aiui_frame(data)
    if frame:
        rospy.logdebug(
            "[SleepCtrl] Received AIUI frame: type=0x%02X id=%d payload=%s",
            frame["type"],
            frame["message_id"],
            frame["payload"].hex(" "),
        )
    else:
        rospy.logdebug("[SleepCtrl] Received non-frame data: %s", data.hex(" "))

    return frame


def _send_handshake(sock, retries=3, interval=0.1):
    for _ in range(retries):
        frame = build_handshake_frame()
        rospy.logdebug("[SleepCtrl] Sending handshake: %s", frame.hex(" "))
        sock.sendall(frame)

        ack = _recv_one_frame(sock)
        if ack and ack["type"] == MSG_TYPE_ACK:
            return True

        time.sleep(interval)

    return False


def _send_one_control_frame(frame, label, host, port, timeout):
    """
    单独建立一次 socket，发送一条控制帧。

    原因：有些 AIUI 固件收到某条控制消息后会主动断开 socket。
    如果复用同一个 socket 连续发送多条消息，第二条或第三条就可能报
    Broken pipe。
    """
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)

        if not _send_handshake(sock):
            rospy.logwarn("[SleepCtrl] No handshake ACK before %s; continuing", label)

        rospy.loginfo("[SleepCtrl] Sending %s frame to %s:%s", label, host, port)
        rospy.logdebug("[SleepCtrl] %s frame: %s", label, frame.hex(" "))
        sock.sendall(frame)
        return _recv_one_frame(sock)


def send_sleep_command(host=AIUI_HOST, port=AIUI_PORT, timeout=SOCKET_TIMEOUT):
    """
    发送休眠控制。

    默认流程：
      1. 可选关闭 AIUI 声音播放开关；
      2. 发送 CMD_RESET_WAKEUP，让盒子进入待唤醒/休眠状态。

    注意：
      - CMD_RESET_WAKEUP 不是重启设备，它是重置唤醒状态；
      - 如果你已经确认 tts-stop 或 tts-cancel 有效，可以取消下面注释继续测试；
      - 如果真正播放来自你们本机的 mp3 播放器，而不是盒子 AIUI 播放通道，
        这里的 socket 命令无法立刻杀掉本机播放器，需要在 TTS 播放节点里停播。
    """
    warnings = []

    # 可选：停止/取消 TTS 播放。
    # 当前先默认注释掉，因为你的日志显示这类命令容易触发 Broken pipe。
    # 如果要测试它们是否能打断播放，请取消下面两个 try 块的注释。
    #
    # try:
    #     _send_one_control_frame(build_tts_stop_frame(), "tts-stop", host, port, timeout)
    # except OSError as exc:
    #     warnings.append("tts-stop failed: {}".format(exc))
    #
    # try:
    #     _send_one_control_frame(build_tts_cancel_frame(), "tts-cancel", host, port, timeout)
    # except OSError as exc:
    #     warnings.append("tts-cancel failed: {}".format(exc))

    # 可选：关闭 AIUI 声音播放开关。
    # 若只想直接休眠，不想额外发送关闭声音消息，请注释掉下面 5 行。
    try:
        _send_one_control_frame(build_voice_enable_frame(False), "stop-voice", host, port, timeout)
    except OSError as exc:
        warnings.append("stop-voice failed: {}".format(exc))

    # 必选：发送休眠命令。这个命令成功，才认为 service 成功。
    try:
        ack = _send_one_control_frame(build_reset_wakeup_frame(), "CMD_RESET_WAKEUP", host, port, timeout)
    except ConnectionRefusedError:
        message = "Connection refused: {}:{}. Check AIUI host/port and socket service.".format(host, port)
        rospy.logerr("[SleepCtrl] %s", message)
        return False, message
    except socket.timeout:
        message = "Socket timeout while connecting/sending to {}:{}".format(host, port)
        rospy.logerr("[SleepCtrl] %s", message)
        return False, message
    except OSError as exc:
        message = "Socket error while sending CMD_RESET_WAKEUP: {}".format(exc)
        rospy.logerr("[SleepCtrl] %s", message)
        return False, message
    except Exception as exc:
        message = "Failed to send AIUI sleep command: {}".format(exc)
        rospy.logerr("[SleepCtrl] %s", message)
        return False, message

    if ack and ack["type"] == MSG_TYPE_ACK:
        if warnings:
            return True, "AIUI sleep command sent and ACK received; warnings: " + "; ".join(warnings)
        return True, "AIUI sleep command sent and ACK received"

    if warnings:
        return True, "AIUI sleep command sent; no final ACK received; warnings: " + "; ".join(warnings)
    return True, "AIUI sleep command sent; no final ACK received"


def handle_sleep_service(_req):
    success, message = send_sleep_command()
    return TriggerResponse(success=success, message=message)


def main():
    global AIUI_HOST, AIUI_PORT, SOCKET_TIMEOUT

    rospy.init_node("aiui_sleep_controller", anonymous=False)

    AIUI_HOST = rospy.get_param("~aiui_host", AIUI_HOST)
    AIUI_PORT = int(rospy.get_param("~aiui_port", AIUI_PORT))
    SOCKET_TIMEOUT = float(rospy.get_param("~socket_timeout", SOCKET_TIMEOUT))

    rospy.Service("/aiui/sleep_control", Trigger, handle_sleep_service)

    rospy.loginfo(
        "[SleepCtrl] Ready. Target AIUI box: %s:%s. Service: /aiui/sleep_control",
        AIUI_HOST,
        AIUI_PORT,
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SleepCtrl] Node shutdown")
