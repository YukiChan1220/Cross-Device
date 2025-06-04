import serial
import struct
import time
import threading

# 配置参数
PORT = 'COM23'
BAUDRATE = 1000000  # 1Mbps波特率
TIMEOUT = 0.1  # 串口读取超时时间
OUTPUT_FILE = 'output.txt'
DATA_BLOCK_SIZE = 56  # 14个浮点数 × 4字节
SEPARATORS = [b'\xA1' * 8, b'\xA2' * 8, b'\xA3' * 8, b'\xA4' * 8]
SEPARATOR_TO_LINE_PREFIX = {SEPARATORS[0]: '1', SEPARATORS[1]: '2', SEPARATORS[2]: '3', SEPARATORS[3]: '4'}

# 全局变量用于计数
line_count = 0

def print_line_count():
    global line_count
    while True:
        time.sleep(1)  # 每秒钟执行一次
        print(f"data rate: {line_count / 4} Hz")
        line_count = 0  # 重置计数器

def main():
    global line_count
    buffer = bytearray()

    try:
        # 初始化串口连接
        with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
            # 复位设备（根据实际需求调整复位方式）
            ser.dtr = True
            time.sleep(0.1)
            ser.dtr = False
            time.sleep(1)  # 等待设备复位完成

            # 发送启动命令
            ser.write("s".encode('utf-8'))
            print("复位完成并已发送启动命令's'")

            # 启动一个线程来每秒打印行数
            threading.Thread(target=print_line_count, daemon=True).start()

            # 持续接收数据
            while True:
                try:
                    # 读取串口数据
                    data = ser.read(256)
                    if not data:
                        continue  # 无数据时继续循环
                    buffer += data

                    # 处理缓冲区数据
                    while True:
                        # 查找分隔符位置
                        sep_pos = -1
                        sep_index = -1
                        for i, sep in enumerate(SEPARATORS):
                            pos = buffer.find(sep)
                            if pos != -1 and (sep_pos == -1 or pos < sep_pos):
                                sep_pos = pos
                                sep_index = i

                        if sep_pos == -1:
                            break  # 未找到分隔符

                        # 检查数据完整性
                        if sep_pos < DATA_BLOCK_SIZE:
                            print(f"数据错误：在位置{sep_pos}发现不完整数据块")
                            buffer = buffer[sep_pos + len(SEPARATORS[0]):]
                            continue

                        # 提取并验证数据块
                        data_block = buffer[sep_pos - DATA_BLOCK_SIZE:sep_pos]
                        if len(data_block) != DATA_BLOCK_SIZE:
                            print(f"数据长度错误：期望{DATA_BLOCK_SIZE}字节，实际{len(data_block)}字节")
                            buffer = buffer[sep_pos + len(SEPARATORS[0]):]
                            continue

                        # 解析浮点数（假设大端字节序）
                        try:
                            floats = struct.unpack('<14f', data_block)
                        except struct.error as e:
                            print(f"数据解析失败：{e}")
                            buffer = buffer[sep_pos + len(SEPARATORS[0]):]
                            continue

                        # 写入文件
                        try:
                            with open(OUTPUT_FILE, 'a') as f:
                                line_prefix = SEPARATOR_TO_LINE_PREFIX[SEPARATORS[sep_index]]
                                line = f"{line_prefix} " + ' '.join(f"{num:.2f}" for num in floats)
                                f.write(line + '\n')
                        except IOError as e:
                            print(f"文件写入失败：{e}")

                        # 更新缓冲区
                        buffer = buffer[sep_pos + len(SEPARATORS[0]):]

                        # 增加行数计数
                        line_count += 1

                except serial.SerialException as e:
                    print(f"串口通信错误：{e}")
                    break

    except serial.SerialException as e:
        print(f"无法打开串口{PORT}：{e}")
    except KeyboardInterrupt:
        print("用户终止程序")
    except Exception as e:
        print(f"未处理的异常：{e}")

if __name__ == "__main__":
    main()
