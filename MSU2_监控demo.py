# -*- coding: utf-8 -*-
"""
MSU2 / MSU2_MINI 可编程 USB 屏幕 —— 纯性能监控 Demo（不投屏）
================================================================
协议依据 MSU2_MINI_DemoV1.6.exe 反汇编逐条核对（非推定）。

功能：连接屏幕后显示 CPU 占用率 / 内存占用率 / 显卡占用率 / 磁盘占用，
      每 0.5 秒刷新一次，CPU 占用远低于原版投屏程序。
      显卡优先用 pynvml 读取，没有则调用 nvidia-smi（支持 NVIDIA 显卡；
      取不到显卡时右下角自动退回显示电池）。

运行前准备：
    pip install pyserial psutil
    # 可选（NVIDIA 显卡，装了驱动一般自带 nvidia-smi）：
    pip install nvidia-ml-py

用法：
    python MSU2_监控demo.py            # 正常方向，右下角显卡占用率
    python MSU2_监控demo.py --flip     # 旋转 180°（屏幕反插/倒放时查看）
    python MSU2_监控demo.py --item battery   # 右下角改回电池电量
    python MSU2_监控demo.py --nv       # NVIDIA 全显卡模式（默认同时 --flip）：
                                       #   左上=GPU核心温度，右上=显存占用%，
                                       #   左下=GPU功率%，右下=GPU利用率

注意：
  1. 右下角标签文字（"电池"）是背景位图里预置的，--item / --nv 只改数字来源；
     想改标签文字需要重烧背景图，随时可以找我做。
  2. --nv 依赖 nvidia-smi（N 卡驱动自带）；取不到数据时自动退回普通监控。
  3. 运行前请关闭"USB屏幕助手V1.0"和原版投屏程序（串口只能被一个程序占用）；
  4. 若提示找不到设备，临时关闭电脑蓝牙后重新插拔屏幕再试；
  5. Win10/11 免驱动，直接插上即可；
  6. 程序每次启动都会显式设置方向：不带 --flip 跑一次即可恢复正常方向。
"""

import time
import threading
import argparse
import serial
import serial.tools.list_ports
import psutil

# ---------------- RGB565 常用颜色（原版 color_now = R*2048 + G*32 + B） ----------------
RED    = 0xF800
GREEN  = 0x07E0
BLUE   = 0x001F
WHITE  = 0xFFFF
BLACK  = 0x0000
YELLOW = 0xFFE0

BAUD = 19200          # 屏幕串口波特率（反汇编确认）

# 闪存页地址（反汇编自 show_PC_state：photo_add=4038, num_add=4026）
PAGE_BG    = 4038     # 160x80 监控背景位图
PAGE_DIGIT = 4026     # 数字 0~9 位图，每页一张，尺寸 24x33（4026~4035）
PAGE_ONE   = 4036     # 百位 "1"（8x33）
PAGE_BLANK = 4037     # 百位空白（8x33，数值不足百位时占位）

FC = BLUE             # 数字颜色（原版监控模式 State_machine=1 用 BLUE）
BC = BLACK            # 背景色

ser = None            # 全局串口对象
ser_lock = threading.Lock()

# 单一读线程机制：只有 reader_loop 读串口，避免和主线程抢字节
ack_data = b''
ack_lock = threading.Lock()
ack_event = threading.Event()


# ==================== 串口底层（与原版一致） ====================

def ser_write(data: bytes) -> bool:
    """发送一帧；串口异常时置 ser=None 并返回 False"""
    global ser
    with ser_lock:
        try:
            if ser is None or not ser.is_open:
                return False
            ser.write(bytes(data))
            return True
        except Exception:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            return False


def ser_read() -> bytes:
    """非阻塞读取全部已到达字节（与原版 SER_Read 相同）。
    注意：仅握手阶段由主线程调用；连接建立后只有 reader_loop 调用。"""
    with ser_lock:
        try:
            if ser is None or not ser.is_open:
                return b''
            n = ser.in_waiting
            return ser.read(n) if n else b''
        except Exception:
            return b''


# ==================== 设备握手（反汇编自 Get_MSN_Device） ====================
# 屏幕会周期性广播 "MSN"+两位数字 的上线通告；
# PC 收到后回复 b'\x00MSNCN'，屏幕回显确认后即建立连接。

def try_handshake(port_name: str, wait_total: float = 6.0) -> bool:
    global ser
    try:
        ser = serial.Serial(port_name, BAUD, timeout=0.1)
    except Exception:
        ser = None
        return False
    time.sleep(0.25)  # 与原版一致，等屏幕首条广播

    buf = b''
    deadline = time.time() + wait_total
    while time.time() < deadline and ser is not None:
        recv = ser_read()
        if recv:
            buf += recv
            # 只保留末尾一小段，防止缓冲无限增长
            if len(buf) > 256:
                buf = buf[-64:]
            idx = buf.find(b'MSN')
            if idx >= 0 and len(buf) >= idx + 5 \
               and 0x30 <= buf[idx + 3] <= 0x39 \
               and 0x30 <= buf[idx + 4] <= 0x39:
                ser_write(b'\x00MSNCN')
                # 等屏幕回显 MSNCN 确认
                hb = b''
                t0 = time.time()
                while time.time() - t0 < 1.0 and ser is not None:
                    recv = ser_read()
                    if recv:
                        hb += recv
                        if b'MSNCN' in hb:
                            return True
                    time.sleep(0.01)
                buf = b''
        time.sleep(0.02)

    try:
        ser.close()
    except Exception:
        pass
    ser = None
    return False


def find_device() -> bool:
    """枚举串口，用 MSN 握手识别屏幕"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print('[!] 未检测到任何串口，请确认屏幕已插入电脑')
        return False
    for p in ports:
        print(f'[..] 正在探测 {p.device} ({p.description}) ...')
        if try_handshake(p.device):
            print(f'[OK] 已连接屏幕: {p.device} ({p.description})')
            return True
    print('[!] 未找到屏幕设备。可尝试：关闭蓝牙 → 重新插拔屏幕 → 重试')
    return False


# ==================== 单一读线程：应答广播 + 分发指令回显 ====================

def reader_loop():
    """连接期间唯一读取串口的线程。
    - 看到 'MSN'+两位数字 的广播 → 回复 b'\\x00MSNCN' 保持连接；
    - 其余字节视为 LCD 指令的回显 → 存起来并通知等待中的主线程。"""
    global ser, ack_data
    buf = b''
    while ser is not None:
        recv = ser_read()
        if not recv:
            time.sleep(0.005)
            continue
        buf += recv
        if len(buf) > 512:
            buf = buf[-128:]
        # 处理所有完整的 MSN 广播
        while True:
            idx = buf.find(b'MSN')
            if idx < 0 or len(buf) < idx + 5:
                break
            if 0x30 <= buf[idx + 3] <= 0x39 and 0x30 <= buf[idx + 4] <= 0x39:
                ser_write(b'\x00MSNCN')
            buf = buf[:idx] + buf[idx + 5:]
        # 剩余字节 = 指令回显
        if buf:
            with ack_lock:
                ack_data += buf
            buf = b''
            ack_event.set()


def wait_ack(timeout=0.5) -> bytes:
    """等待读线程收到回显（原版 LCD_Photo_wb 会等回显并核对帧头）"""
    global ack_data
    ack_event.clear()
    ack_event.wait(timeout)
    with ack_lock:
        d = ack_data
        ack_data = b''
    return d


# ==================== LCD 指令（反汇编核对，6 字节定长帧） ====================

def lcd_set_xy(x: int, y: int):
    """设置光标坐标，帧: [2, 0, x_hi, x_lo, y_hi, y_lo]"""
    ser_write(bytes([2, 0, (x // 256) & 0xFF, x % 256,
                     (y // 256) & 0xFF, y % 256]))


def lcd_set_size(w: int, h: int):
    """设置显示区域尺寸，帧: [2, 1, w_hi, w_lo, h_hi, h_lo]"""
    ser_write(bytes([2, 1, (w // 256) & 0xFF, w % 256,
                     (h // 256) & 0xFF, h % 256]))


def lcd_set_color(fc: int, bc: int):
    """设置前景/背景色，帧: [2, 2, fc_hi, fc_lo, bc_hi, bc_lo]"""
    ser_write(bytes([2, 2, (fc // 256) & 0xFF, fc % 256,
                     (bc // 256) & 0xFF, bc % 256]))


def lcd_set_state(state: int) -> bool:
    """设置显示方向（原版 LCD_State，"切换显示方向"按钮调用）。
    帧: [2, 3, 10, state, 0, 0]；
    方向只有两档（原版 LCD_Change 在 0/1 间循环）：
      0 = 正常（0°），1 = 翻转（180°）。
    注意：其它值（2/3）会进入不支持的 90° 模式，造成花屏。"""
    if not ser_write(bytes([2, 3, 10, state & 0xFF, 0, 0])):
        return False
    time.sleep(0.001)
    ack = wait_ack(0.5)
    return len(ack) >= 2 and ack[0] == 2 and ack[1] == 3


def lcd_photo_wb(x, y, x_size, y_size, page_add, fc, bc) -> bool:
    """显示一幅闪存中的单色位图（原版 LCD_Photo_wb）。
    帧: [2, 3, 1, page_hi, page_lo, 0]，随后等待帧头回显 [2, 3]"""
    lcd_set_xy(x, y)
    lcd_set_size(x_size, y_size)
    lcd_set_color(fc, bc)
    if not ser_write(bytes([2, 3, 1, (page_add // 256) & 0xFF,
                            page_add % 256, 0])):
        return False
    time.sleep(0.001)
    ack = wait_ack(0.5)
    # 原版：回显首两字节不符即认为掉线
    return len(ack) >= 2 and ack[0] == 2 and ack[1] == 3


# ==================== 监控界面（布局与原版 show_PC_state 一致） ====================

def draw_background():
    """绘制 160x80 监控背景位图（闪存第 4038 页）"""
    return lcd_photo_wb(0, 0, 160, 80, PAGE_BG, FC, BC)


def draw_value(value: int, x: int, y: int) -> bool:
    """在 (x,y) 处绘制 0~199 的百分数数字（原版布局逻辑）。
    百位 8 宽，十位/个位 24 宽，高 33"""
    value = max(0, min(199, int(value)))
    if value >= 100:
        if not lcd_photo_wb(x, y, 8, 33, PAGE_ONE, FC, BC):
            return False
        value %= 100
    else:
        if not lcd_photo_wb(x, y, 8, 33, PAGE_BLANK, FC, BC):
            return False
    if not lcd_photo_wb(x + 8,  y, 24, 33, PAGE_DIGIT + value // 10, FC, BC):
        return False
    if not lcd_photo_wb(x + 32, y, 24, 33, PAGE_DIGIT + value % 10, FC, BC):
        return False
    return True


# ==================== 显卡占用率采集 ====================

_nvml_ok = None   # None=未尝试, True=可用, False=不可用


def get_gpu_percent():
    """取 GPU 占用率（%）。优先 pynvml，其次 nvidia-smi；取不到返回 None。
    支持 NVIDIA 显卡；AMD/Intel 核显没装对应工具时返回 None。"""
    global _nvml_ok
    if _nvml_ok is not False:
        try:
            import pynvml
            if _nvml_ok is None:
                pynvml.nvmlInit()
                _nvml_ok = True
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            return int(pynvml.nvmlDeviceGetUtilizationRates(handle).gpu)
        except Exception:
            _nvml_ok = False
    try:
        import subprocess
        out = subprocess.check_output(
            ['nvidia-smi', '--query-gpu=utilization.gpu',
             '--format=csv,noheader,nounits'],
            timeout=3)
        return int(out.decode('ascii', errors='ignore').strip().splitlines()[0])
    except Exception:
        return None


def get_battery_percent():
    """取电池电量（%），无电池设备返回 100（与原版一致）"""
    battery = psutil.sensors_battery()
    return int(battery.percent) if battery else 100


def get_nv_metrics():
    """取 NVIDIA 显卡四项数据，返回 (util, temp, mem_pct, power_pct)，
    取不到返回 None。
      util      = GPU 利用率 %
      temp      = 核心温度 ℃
      mem_pct   = 显存占用 %（已用/总量）
      power_pct = 功率 %（当前功耗/功耗上限）
    优先 pynvml，其次 nvidia-smi（N 卡驱动自带）。"""
    global _nvml_ok
    if _nvml_ok is not False:
        try:
            import pynvml
            if _nvml_ok is None:
                pynvml.nvmlInit()
                _nvml_ok = True
            h = pynvml.nvmlDeviceGetHandleByIndex(0)
            util = pynvml.nvmlDeviceGetUtilizationRates(h).gpu
            temp = pynvml.nvmlDeviceGetTemperature(h, pynvml.NVML_TEMPERATURE_GPU)
            mem = pynvml.nvmlDeviceGetMemoryInfo(h)
            mem_pct = mem.used * 100.0 / mem.total if mem.total else 0
            draw = pynvml.nvmlDeviceGetPowerUsage(h)      # mW
            try:
                limit = pynvml.nvmlDeviceGetPowerLimit(h)  # mW
            except AttributeError:
                limit = pynvml.nvmlDeviceGetEnforcedPowerLimit(h)
            power_pct = draw * 100.0 / limit if limit else 0
            return int(util), int(temp), int(mem_pct), int(power_pct)
        except Exception:
            _nvml_ok = False
    try:
        import subprocess
        out = subprocess.check_output(
            ['nvidia-smi',
             '--query-gpu=utilization.gpu,temperature.gpu,'
             'memory.used,memory.total,power.draw,power.limit',
             '--format=csv,noheader,nounits'],
            timeout=3)
        line = out.decode('ascii', errors='ignore').strip().splitlines()[0]
        util, temp, mem_used, mem_total, pwr_draw, pwr_limit = \
            (float(x) for x in (s.strip() for s in line.split(',')))
        mem_pct = mem_used * 100.0 / mem_total if mem_total else 0
        power_pct = pwr_draw * 100.0 / pwr_limit if pwr_limit else 0
        return int(util), int(temp), int(mem_pct), int(power_pct)
    except Exception:
        return None


# ==================== 主程序 ====================

def main():
    parser = argparse.ArgumentParser(
        description='MSU2 USB 屏幕性能监控 Demo（不投屏）')
    parser.add_argument('--flip', action='store_true',
                        help='画面旋转 180°，方便屏幕反向放置时查看')
    parser.add_argument('--item', choices=['gpu', 'battery'], default='gpu',
                        help='右下角显示内容：gpu=显卡占用率（默认），'
                             'battery=电池电量；选 gpu 但检测不到显卡时自动退回电池')
    parser.add_argument('--nv', action='store_true',
                        help='NVIDIA 全显卡模式：核心温度/显存%/功率%/利用率'
                             '四宫格（默认同时开启 --flip）')
    args = parser.parse_args()
    if args.nv:
        args.flip = True   # --nv 默认翻转，便于屏幕倒扣在显卡旁查看

    if not find_device():
        return
    time.sleep(0.2)

    # 启动唯一读线程后再开始发指令
    reader = threading.Thread(target=reader_loop, daemon=True)
    reader.start()

    # 每次启动都显式设置方向：既保证本次朝向正确，也能把屏幕从
    # 其它方向（比如误设的 90° 花屏状态）恢复回来
    state = 1 if args.flip else 0
    if lcd_set_state(state):
        print('[OK] 显示方向: ' + ('180°（翻转）' if args.flip else '0°（正常）'))
    else:
        print('[!] 方向设置失败，沿用屏幕当前方向')

    if not draw_background():
        print('[!] 背景绘制失败，屏幕无应答，请重新插拔后重试')
        return
    print('[OK] 监控已启动，按 Ctrl+C 退出')

    try:
        while ser is not None:
            if args.nv:
                nv = get_nv_metrics()
            else:
                nv = None

            if nv is not None:
                # --nv 模式：四宫格全部来自 NVIDIA 显卡
                util, temp, mem_pct, power_pct = nv
                cpu, ram, gpu, disk = temp, mem_pct, util, power_pct
            else:
                if args.nv:
                    print('[!] 显卡数据获取失败，本轮回退普通监控')
                cpu = int(psutil.cpu_percent(interval=0.5))
                ram = int(psutil.virtual_memory().percent)
                if args.item == 'battery':
                    gpu = get_battery_percent()
                else:
                    gpu = get_gpu_percent()
                    if gpu is None:
                        # 取不到显卡（非 N 卡/未装工具）→ 退回显示电池
                        gpu = get_battery_percent()
                disk = int(psutil.disk_usage('/').used * 100
                           / psutil.disk_usage('/').total)

            # 与原版 show_PC_state 布局一致：
            #   左上 CPU，右上内存；左下磁盘，右下电池（--item/--nv 时换数据源）
            ok = True
            ok &= draw_value(min(cpu, 199), 24,  0)
            ok &= draw_value(min(ram, 199), 104, 0)
            ok &= draw_value(max(gpu, 0),   104, 47)
            ok &= draw_value(min(disk, 199), 24, 47)
            if nv is not None:
                time.sleep(0.4)  # nv 模式没有 psutil 的 0.5s 采样间隔， pacing 到约 0.5s
            if not ok:
                print('[!] 屏幕无应答，连接已断开')
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        print('已退出')


if __name__ == '__main__':
    main()
