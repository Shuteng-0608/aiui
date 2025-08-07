import time
from typing import List, Callable
from collections.abc import Iterable
from queue import Queue, Full
from pynput.keyboard import Key, Listener

class KeyboardListener:
    def __init__(self):
        self._status = {}
        self._press_time = {}
        self._release_time = {}
        self._press_continue_time = {}
        self._key_events = {}
        self._event_queues = {}
        self._listener = Listener(
            on_press=self._on_press,
            on_release=self._on_release,
            daemon=True)

    def start(self):
        self._listener.start()

    def stop(self):
        self._listener.stop()

    def join(self, timeout=None, *args):
        self._listener.join(timeout=timeout, *args)

    def event_register(self, idx, event_type:int=0, callbacks:List[Callable]=None, queues:List[Queue]=None):
        """按键事件触发机制注册
            type:
                0:  按键按下事件
                1： 按键松开事件
                2： 按键按下和松开事件
        """
        # 检测可迭代对象（排除字符串）
        if isinstance(idx, Iterable) and not isinstance(idx, str):
            for single_key in idx:
                self._register_single_event(single_key, event_type, callbacks, queues)
        else:
            self._register_single_event(idx, event_type, callbacks, queues)

    def _register_single_event(self, key, event_type, callback, queues):
        """为单个键注册事件"""
        self._key_events[key] = {
            "type": event_type,
            "callbacks": callback,
            "queues": queues
        }

    def _on_press(self, key):
        idx = self._get_key_index(key)
        if not self._status.get(idx, False): # 防止重复执行
            self._status[idx] = True
            self._press_time[idx] = time.time()
            self._event_handler(idx, True)

    def _on_release(self, key):
        idx = self._get_key_index(key)
        self._status[idx] = False
        self._event_handler(idx, False)
        if key == Key.esc:
            self.stop()
            print("[Keyboard] 已停止")
            return False

    def _event_handler(self, idx, action_type):
        if idx in self._key_events: # 若按键注册了事件
            if self._key_events[idx]["type"] == 0 and self._status[idx] == True:  # 如果目标事件类型按下类型,且按下
                print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                print(f' - 按下 [{idx}]')
                self._execute_callbacks(idx, 0)
                self._send_queues(idx, True)

            elif self._key_events[idx]["type"] == 1 and self._status[idx] == False:  # 如果目标事件类型是松开类型，且松开
                print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                self._release_time[idx] = time.time()
                self._press_continue_time[idx] = self._release_time[idx] - self._press_time[idx]
                print(f' - 松开 [{idx}], 共计持续 {self._press_continue_time[idx]:.3f} (s)')
                self._execute_callbacks(idx, 0)
                self._send_queues(idx, False)

            elif self._key_events[idx]["type"] == 2:  # 如果目标事件类型是长按类型
                print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                if action_type: # 当按下按键时
                    self._execute_callbacks(idx, 0)
                    self._send_queues(idx, True)
                else: # 当松开按键时
                    self._execute_callbacks(idx, 1)
                    self._send_queues(idx, False)

    def _execute_callbacks(self, idx, callback_id):
        if self._key_events[idx]['callbacks'] is not None:
            self._key_events[idx]['callbacks'][callback_id](idx)

    def _send_queues(self, idx, msg):
        if self._key_events[idx]['queues'] is not None:
            for queue in self._key_events[idx]['queues']:
                try:
                    queue.put(msg, block=False)
                except Full:
                    print(f" - [Keyboard] 触发事件队列 Full 异常")
                except Exception as err:
                    print(f" - [Keyboard] 触发未捕获异常：{err}")

    @staticmethod
    def _get_key_index(key):
        """统一键标识符格式"""
        try:
            return key.char  # 普通字符键
        except AttributeError:
            return key.name  # 特殊功能键



if __name__ == '__main__':
    test = KeyboardListener()
    q1 = Queue(maxsize=1)
    q2 = Queue(maxsize=1)

    test.event_register('a',
                        0,
                        [lambda _:print(" - 执行按键按下事件 [a]")],
                        [q1, q2])
    test.event_register('s',
                        1,
                        [lambda _:print(" - 执行按键松开事件 [s]")],
                        [q1])
    test.event_register(['1', '2', '3', '4'],
                        0,
                        [lambda _:print(" - 执行按键按下事件 [number]")],
                        [q1])
    test.event_register(Key.space.name,
                        2,
                        [lambda _: print(" - 执行按键按下事件 [space]"), lambda _: print(" - 执行按键松开事件 [space]")],
                        [q1])
    # 主线程
    try:
        test.start()
        print("[Keyboard] 已启动线程")
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        print("[Keyboard] 收到终止信号")
    except Exception as e:
        print(f"[Keyboard] 触发错误：{e}")
    finally:
        test.stop()
        print("[Keyboard] 正常结束")
