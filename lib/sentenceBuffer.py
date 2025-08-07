import re
import queue
import threading
import time

# SentenceBuffer 类用于处理文本缓冲区，提取完整句子
# 支持多线程安全操作，自动处理句子分割和缓冲区管理
class SentenceBuffer:
    def __init__(self, min_sentence_length=2, max_sentence_length=50):
        self.buffer = ""
        self.min_sentence_length = min_sentence_length  # 句子最小长度
        self.max_sentence_length = max_sentence_length  # 句子最大长度
        self.sentence_queue = queue.Queue()
        self.lock = threading.Lock()
        
        # 启动句子处理线程
        self.process_thread = threading.Thread(target=self.process_buffer)
        self.process_thread.daemon = True
        self.process_thread.start()
    
    def append_text(self, text):
        """将新的文本片段添加到缓冲区"""
        with self.lock:
            self.buffer += text
            # 通知处理线程有新数据
            threading.Thread(target=self._trigger_processing).start()
    
    def _trigger_processing(self):
        """触发缓冲区处理（线程安全）"""
        with self.lock:
            pass  # 仅用于触发处理线程
    
    def process_buffer(self):
        """在后台处理缓冲区，提取完整句子"""
        while True:
            with self.lock:
                # 检查是否有足够的文本用于句子分割
                if len(self.buffer) >= self.min_sentence_length:
                    # 寻找最佳断句点
                    sentence, remaining = self.find_best_breakpoint(self.buffer)
                    
                    if sentence:
                        # 提取完整句子加入队列
                        self.sentence_queue.put(sentence)
                        self.buffer = remaining
                    elif len(self.buffer) >= self.max_sentence_length:
                        # 强制断句（避免过长句子）
                        self.sentence_queue.put(self.buffer[:self.max_sentence_length])
                        self.buffer = self.buffer[self.max_sentence_length:]
            
            # 暂停以避免高CPU使用率
            time.sleep(0.1)
    
    def find_best_breakpoint(self, text):
        """在文本中找到最佳断句点"""
        # 1. 首先寻找结束标点
        end_punctuation_pattern = r'[。！？.!?;:；：…]'
        match = re.search(end_punctuation_pattern, text[self.min_sentence_length:])
        
        if match:
            # 找到了标点，在标点后断句
            end_pos = self.min_sentence_length + match.end()
            return text[:end_pos], text[end_pos:]
        
        # 2. 其次寻找逗号、分句号等
        mid_punctuation_pattern = r'[,，、|]'
        match = re.search(mid_punctuation_pattern, text[self.min_sentence_length:])
        
        if match:
            # 在逗号后断句
            end_pos = self.min_sentence_length + match.end()
            return text[:end_pos], text[end_pos:]
        
        # 3. 寻找自然停顿点（如空格、连词等）
        natural_break_pattern = r'[\s]|(而且)|(然后)|(所以)|(因为)|(例如)'
        match = re.search(natural_break_pattern, text[self.min_sentence_length:])
        
        if match:
            end_pos = self.min_sentence_length + match.end()
            return text[:end_pos], text[end_pos:]
        
        # 4. 没有找到合适的断句点
        return None, None
    
    def flush(self):
        """处理缓冲区的剩余内容"""
        with self.lock:
            if self.buffer:
                self.sentence_queue.put(self.buffer)
                self.buffer = ""
    
    def get_next_sentence(self, timeout=1):
        """从队列中获取下一个句子"""
        try:
            return self.sentence_queue.get(timeout=timeout)
        except queue.Empty:
            return None