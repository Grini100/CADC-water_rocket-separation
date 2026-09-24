#!/usr/bin/env python3
"""
水火箭飞行数据分析工具
用法: python flight_analyzer.py <数据文件.bin>

数据文件要求:
  - 串口助手用 HEX/二进制 模式保存（不是ASCII文本模式）
  - 文件内容 = ASCII包头 "WLOG,NNNN\\n" + NNNN x 32字节二进制条目

条目格式(32B, 小端):
  [0..3]  uint32 Tick      起飞后毫秒数
  [4]     uint8  State     0=IDLE 1=ASCENT 2=COAST 3=SEP_DONE 4=DONE
  [5]     uint8  Flags     bit0=I2C失败冻结旧值
  [6..7]  int16  Ax        原始LSB (2048 LSB/g)
  [8..9]  int16  Ay
  [10..11]int16  Az
  [12..13]int16  Gx        原始LSB (16.4 LSB/deg/s)
  [14..15]int16  Gy
  [16..17]int16  Gz
  [18..19]uint16 Mag100    |a| x 100 (单位g)
  [20..31]0xFF 保留
"""

import sys
import os
import struct

# ============================================================
# 自动安装缺失的库
# ============================================================
def ensure_package(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
        return True
    except ImportError:
        print(f"[安装] 正在安装 {package} ...")
        ret = os.system(f'{sys.executable} -m pip install {package}')
        if ret == 0:
            print(f"[安装] {package} 安装成功")
            return True
        else:
            print(f"[安装] {package} 安装失败，请手动运行: pip install {package}")
            return False

has_matplotlib = ensure_package('matplotlib')
has_numpy = ensure_package('numpy')

if has_matplotlib:
    import matplotlib.pyplot as plt
    # Windows 中文显示
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial']
    plt.rcParams['axes.unicode_minus'] = False

if has_numpy:
    import numpy as np


# ============================================================
# 数据解析
# ============================================================
STATE_NAMES = {
    0: 'IDLE',
    1: 'ASCENT (动力上升)',
    2: 'COAST (滑行)',
    3: 'SEP_DONE (已分离)',
    4: 'DONE (已开伞)',
}

STATE_COLORS = {
    0: '#888888',
    1: '#FF4444',
    2: '#4444FF',
    3: '#44AA44',
    4: '#AAAAAA',
}


def parse_file(filepath):
    """读取 .bin 文件，解析为条目列表"""
    with open(filepath, 'rb') as f:
        raw = f.read()

    # 尝试找 WLOG 包头（raw binary 模式）
    idx = raw.find(b'WLOG,')
    if idx == -1:
        # 可能是 HEX 文本格式，尝试转换
        print("[提示] 未找到 WLOG 包头，尝试解析 HEX 文本格式...")
        raw = hex_text_to_bytes(raw)
        idx = raw.find(b'WLOG,')

    if idx == -1:
        print("[错误] 文件中未找到 WLOG 包头！")
        print("请确认串口助手以 HEX/二进制 模式保存，且按了 KEY2 触发 dump")
        return None

    # 解析包头: WLOG,NNNN\n
    header_end = raw.find(b'\n', idx)
    if header_end == -1:
        print("[错误] 包头不完整（缺少换行符）")
        return None

    header_str = raw[idx:header_end].decode('ascii', errors='replace')
    try:
        count = int(header_str.split(',')[1])
    except (ValueError, IndexError):
        print(f"[错误] 无法解析条数: {header_str}")
        return None

    print(f"包头: {header_str}")
    print(f"声明条数: {count}")

    # 定位数据起始
    data_start = header_end + 1
    expected_bytes = count * 32
    available = len(raw) - data_start
    if available < expected_bytes:
        actual = available // 32
        print(f"[警告] 文件数据不足: 期望 {expected_bytes} 字节, 实际 {available} 字节")
        print(f"       将按实际可用条数 {actual} 解析")
        count = actual

    data = raw[data_start:data_start + count * 32]

    # 逐条解析
    entries = []
    for i in range(count):
        e = data[i * 32:(i + 1) * 32]
        if len(e) < 32:
            break

        tick = struct.unpack_from('<I', e, 0)[0]
        state = e[4]
        flags = e[5]
        ax, ay, az = struct.unpack_from('<hhh', e, 6)
        gx, gy, gz = struct.unpack_from('<hhh', e, 12)
        mag100 = struct.unpack_from('<H', e, 18)[0]

        # 跳过全 0xFF 的补位条目
        if tick == 0xFFFFFFFF and mag100 == 0xFFFF:
            continue

        entries.append({
            'tick': tick,
            'time': tick / 1000.0,
            'state': state,
            'flags': flags,
            'i2c_err': bool(flags & 0x01),
            'ax': ax, 'ay': ay, 'az': az,
            'ax_g': ax / 2048.0,
            'ay_g': ay / 2048.0,
            'az_g': az / 2048.0,
            'gx': gx, 'gy': gy, 'gz': gz,
            'gx_dps': gx / 16.4,
            'gy_dps': gy / 16.4,
            'gz_dps': gz / 16.4,
            'mag': mag100 / 100.0,
        })

    print(f"有效条目: {len(entries)}")
    return entries


def hex_text_to_bytes(raw):
    """把 HEX 文本格式 (如 '57 4C 4F 47...') 转为原始字节
    兼容多种串口助手格式：纯hex、带RX:前缀、带时间戳、带ASCII注释等
    """
    import re
    try:
        text = raw.decode('ascii', errors='ignore')
    except Exception:
        return raw

    # 方法1: 匹配空格/行首分隔的2位hex值（最常见格式）
    # 用正则确保只取被空白/行首/行尾包围的独立hex对，
    # 避免从 "WLOG" 这类ASCII词里误匹配 "4C" 等
    clean_pairs = []
    for m in re.finditer(
        r'(?:^|\s)([0-9A-Fa-f]{2})(?=\s|$)',
        text, re.MULTILINE
    ):
        clean_pairs.append(m.group(1))

    if clean_pairs:
        try:
            result = bytes(int(h, 16) for h in clean_pairs)
            print(f"[HEX文本] 提取到 {len(clean_pairs)} 个字节")
            return result
        except ValueError:
            pass

    # 方法2: 兜底——移除所有非hex字符后直接转换
    hex_str = re.sub(r'[^0-9A-Fa-f]', '', text)
    if len(hex_str) >= 2:
        if len(hex_str) % 2 != 0:
            hex_str = hex_str[:-1]
        try:
            result = bytes.fromhex(hex_str)
            print(f"[HEX文本] 兜底提取到 {len(result)} 个字节")
            return result
        except ValueError:
            pass

    print("[HEX文本] 转换失败，返回原始数据")
    return raw


# ============================================================
# 统计分析
# ============================================================
def find_segments(entries):
    """找到连续相同状态的段"""
    segments = []
    for e in entries:
        s = e['state']
        if not segments or segments[-1]['state'] != s:
            segments.append({
                'state': s,
                'start_tick': e['tick'],
                'end_tick': e['tick'],
                'start_time': e['time'],
                'end_time': e['time'],
                'count': 1,
            })
        else:
            seg = segments[-1]
            seg['end_tick'] = e['tick']
            seg['end_time'] = e['time']
            seg['count'] += 1
    return segments


def analyze(entries):
    """打印飞行数据统计"""
    if not entries:
        print("[错误] 无有效数据")
        return None

    times = [e['time'] for e in entries]
    mags = [e['mag'] for e in entries]
    states = [e['state'] for e in entries]

    # 基本统计
    total_time = times[-1] - times[0]
    i2c_errors = sum(1 for e in entries if e['i2c_err'])
    max_mag = max(mags)
    min_mag = min(mags)
    max_idx = mags.index(max_mag)
    min_idx = mags.index(min_mag)

    print("\n" + "=" * 60)
    print("              飞行数据统计报告")
    print("=" * 60)

    print(f"\n[总览]")
    print(f"  总条目数:     {len(entries)}")
    print(f"  飞行时长:     {total_time:.3f} 秒 ({total_time * 1000:.0f} ms)")
    print(f"  采样间隔:     {total_time / max(len(entries) - 1, 1) * 1000:.1f} ms (理论 10ms)")
    print(f"  I2C 错误条数: {i2c_errors} ({i2c_errors * 100 / len(entries):.1f}%)")

    print(f"\n[加速度峰值/谷值]")
    print(f"  最大 |a|:     {max_mag:.2f} g  @ T+{times[max_idx]:.3f}s (状态={STATE_NAMES.get(states[max_idx], '?')})")
    print(f"  最小 |a|:     {min_mag:.2f} g  @ T+{times[min_idx]:.3f}s (状态={STATE_NAMES.get(states[min_idx], '?')})")

    # 状态段分析
    segments = find_segments(entries)
    print(f"\n[状态段时间线] ({len(segments)} 段)")
    print("-" * 60)
    for i, seg in enumerate(segments):
        sname = STATE_NAMES.get(seg['state'], f"UNKNOWN({seg['state']})")
        dur = seg['end_time'] - seg['start_time']
        seg_mags = [e['mag'] for e in entries if e['state'] == seg['state']
                    and seg['start_tick'] <= e['tick'] <= seg['end_tick']]
        peak = max(seg_mags) if seg_mags else 0
        valley = min(seg_mags) if seg_mags else 0
        mean_mag = sum(seg_mags) / len(seg_mags) if seg_mags else 0
        print(f"  段{i}: {sname}")
        print(f"        T+{seg['start_time']:.3f}s ~ T+{seg['end_time']:.3f}s  "
              f"时长 {dur:.3f}s  条数 {seg['count']}")
        print(f"        |a|: 峰值={peak:.2f}g  谷值={valley:.2f}g  均值={mean_mag:.2f}g")

    # 关键事件
    print(f"\n[关键事件检测]")
    # 起飞点: 第一个 state=1 的条目
    ascent_start = next((e for e in entries if e['state'] == 1), None)
    if ascent_start:
        print(f"  起飞 (→ASCENT):   T+{ascent_start['time']:.3f}s  |a|={ascent_start['mag']:.2f}g")

    # 动力耗尽: 第一个 state=2 的条目
    coast_start = next((e for e in entries if e['state'] == 2), None)
    if coast_start:
        print(f"  动力耗尽 (→COAST): T+{coast_start['time']:.3f}s  |a|={coast_start['mag']:.2f}g")
        burn_time = coast_start['time'] - (ascent_start['time'] if ascent_start else 0)
        print(f"  动力段时长:       {burn_time:.3f}s")

    # 分离: 第一个 state=3 的条目
    sep_start = next((e for e in entries if e['state'] == 3), None)
    if sep_start:
        print(f"  分离 (→SEP_DONE):  T+{sep_start['time']:.3f}s  |a|={sep_start['mag']:.2f}g")
        if coast_start:
            coast_time = sep_start['time'] - coast_start['time']
            print(f"  滑行段时长:       {coast_time:.3f}s")

    # 开伞: 第一个 state=4 的条目
    done_start = next((e for e in entries if e['state'] == 4), None)
    if done_start:
        print(f"  开伞 (→DONE):     T+{done_start['time']:.3f}s  |a|={done_start['mag']:.2f}g")
        if sep_start:
            sep_time = done_start['time'] - sep_start['time']
            print(f"  分离→开伞延时:   {sep_time:.3f}s (程序设定 3.000s)")

    # 失重检测 (COAST 段内 |a| < 0.3g)
    weightless = [e for e in entries if e['state'] == 2 and e['mag'] < 0.3]
    if weightless:
        wz_start = weightless[0]
        print(f"  失重触发 (|a|<0.3g): T+{wz_start['time']:.3f}s  |a|={wz_start['mag']:.2f}g")
        print(f"  失重条数:          {len(weightless)}")
    else:
        print(f"  失重触发:          未检测到 (COAST 段 |a| 始终 >= 0.3g，靠 15s 超时分离)")

    print("\n" + "=" * 60)
    return segments


# ============================================================
# 高度估算（加速度二次积分，无气压计，仅供参考）
# ============================================================
def estimate_altitude(entries):
    """在 [起飞 → 失重] 区间内对合加速度二次积分，估算最大高度。

    原理:
      - 积分区间: 从 |a|>1.8g (起飞) 到 |a|<0.3g (失重=顶点)
      - 区间内净竖直加速度 a_net = (|a| - 1g) * 9.81
        动力段 |a|>1g 加速上升；滑行段 |a|→0 等价 -1g 减速，物理自洽
      - 边界: v(起飞)=0, h(起飞)=0；积分到失重点，h(失重) 即最大高度
      - 不做速度零偏校准；终点速度残差 v(失重) 理想为 0，作为误差指示

    返回 dict: launch_time, apex_time, rise_time, max_h, max_v, v_residual,
               plot_times, plot_hs (区间内曲线，用于绘图)
    """
    G = 9.81
    n = len(entries)
    times = [e['time'] for e in entries]
    mags = [e['mag'] for e in entries]

    # ---- 起飞点：第一条 |a|>1.8g ----
    launch_idx = None
    for i in range(n):
        if mags[i] > 1.8:
            launch_idx = i
            break
    if launch_idx is None:
        print("\n[高度估算] 未检测到起飞 (|a|>1.8g)，跳过")
        return None

    # ---- 失重点：起飞后第一条 |a|<0.3g ----
    apex_idx = None
    for i in range(launch_idx + 1, n):
        if mags[i] < 0.3:
            apex_idx = i
            break
    if apex_idx is None:
        # 未检测到失重（超时分离）：用 state 2→3 跳变点
        for i in range(launch_idx + 1, n):
            if entries[i]['state'] == 3:
                apex_idx = i
                break
    if apex_idx is None:
        print("\n[高度估算] 未检测到失重/分离点，跳过")
        return None

    # ---- 区间内二次积分：v(起飞)=0, h(起飞)=0 ----
    vs = [0.0]
    hs = [0.0]
    for i in range(launch_idx + 1, apex_idx + 1):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            dt = 0.01
        a0 = (mags[i - 1] - 1.0) * G
        a1 = (mags[i] - 1.0) * G
        v_new = vs[-1] + 0.5 * (a0 + a1) * dt      # 梯形积分速度
        hs.append(hs[-1] + 0.5 * (vs[-1] + v_new) * dt)  # 梯形积分高度
        vs.append(v_new)

    max_h = hs[-1]              # 失重点高度 = 最大高度
    max_v = max(vs)
    v_residual = vs[-1]         # 理想为 0（顶点速度=0），残差=累积误差指示
    rise_time = times[apex_idx] - times[launch_idx]

    plot_times = times[launch_idx:apex_idx + 1]
    plot_hs = hs

    print(f"\n[高度估算] (起飞→失重区间积分，无气压计，仅供趋势参考)")
    print(f"  起飞时刻:       T+{times[launch_idx]:.3f}s  |a|={mags[launch_idx]:.2f}g")
    print(f"  失重时刻:       T+{times[apex_idx]:.3f}s  |a|={mags[apex_idx]:.2f}g")
    print(f"  上升用时:       {rise_time:.3f}s")
    print(f"  估算最大高度:   {max_h:.1f} m")
    print(f"  最大上升速度:   {max_v:.1f} m/s")
    print(f"  顶点速度残差:   {v_residual:+.2f} m/s (理想为0，绝对值大说明误差大)")

    return {
        'launch_time': times[launch_idx], 'apex_time': times[apex_idx],
        'rise_time': rise_time, 'max_h': max_h, 'max_v': max_v,
        'v_residual': v_residual,
        'plot_times': plot_times, 'plot_hs': plot_hs,
    }


# ============================================================
# 绘图
# ============================================================
def plot_data(entries, segments, alt=None):
    if not has_matplotlib:
        print("\n[跳过绘图] matplotlib 未安装")
        return

    times = [e['time'] for e in entries]
    mags = [e['mag'] for e in entries]
    ax_g = [e['ax_g'] for e in entries]
    ay_g = [e['ay_g'] for e in entries]
    az_g = [e['az_g'] for e in entries]
    gx_dps = [e['gx_dps'] for e in entries]
    gy_dps = [e['gy_dps'] for e in entries]
    gz_dps = [e['gz_dps'] for e in entries]

    # 有高度数据时 4 行子图，否则保持 3 行
    if alt:
        fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True,
                                 gridspec_kw={'height_ratios': [2, 1.5, 1, 1]})
    else:
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True,
                                 gridspec_kw={'height_ratios': [2, 1, 1]})

    # ---------- 子图1: |a| 合加速度 ----------
    ax1 = axes[0]
    ax1.plot(times, mags, color='#333333', linewidth=1.2, label='|a|')

    # 状态背景色带
    for seg in segments:
        color = STATE_COLORS.get(seg['state'], '#DDDDDD')
        ax1.axvspan(seg['start_time'], seg['end_time'], alpha=0.12, color=color)

    # 阈值参考线
    ax1.axhline(y=1.0, color='green', linestyle=':', linewidth=0.8, alpha=0.6, label='1.0g (静止)')
    ax1.axhline(y=0.3, color='orange', linestyle=':', linewidth=0.8, alpha=0.6, label='0.3g (失重触发)')
    ax1.axhline(y=1.8, color='red', linestyle=':', linewidth=0.8, alpha=0.6, label='1.8g (起飞阈值)')

    # 标注峰值和谷值
    max_idx = mags.index(max(mags))
    min_idx = mags.index(min(mags))
    ax1.annotate(f'峰值 {max(mags):.2f}g', xy=(times[max_idx], mags[max_idx]),
                xytext=(times[max_idx] + 0.5, mags[max_idx]),
                arrowprops=dict(arrowstyle='->', color='red'), fontsize=9, color='red')
    ax1.annotate(f'谷值 {min(mags):.2f}g', xy=(times[min_idx], mags[min_idx]),
                xytext=(times[min_idx] + 0.5, min(mags[min_idx] + 0.5, 0.5)),
                arrowprops=dict(arrowstyle='->', color='blue'), fontsize=9, color='blue')

    # 状态转换竖线 + 标签
    for seg in segments:
        if seg['start_time'] > 0:
            ax1.axvline(x=seg['start_time'], color='gray', linestyle='--', linewidth=0.5)
        sname = STATE_NAMES.get(seg['state'], '?').split(' ')[0]
        mid = (seg['start_time'] + seg['end_time']) / 2
        ax1.text(mid, ax1.get_ylim()[1] if ax1.get_ylim()[1] > 0 else max(mags) * 1.1,
                sname, ha='center', va='top', fontsize=7, alpha=0.5)

    ax1.set_ylabel('合加速度 |a| (g)')
    ax1.set_title('水火箭飞行数据')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ---------- 子图2: 高度估算（有 alt 时插入，只画 起飞→失重 上升段） ----------
    if alt:
        axh = axes[1]
        axh.plot(alt['plot_times'], alt['plot_hs'], color='#8E44AD',
                 linewidth=1.5, label='估算高度 (上升段)')
        axh.fill_between(alt['plot_times'], 0, alt['plot_hs'],
                         alpha=0.1, color='#8E44AD')
        axh.axhline(y=0, color='gray', linewidth=0.5)
        for seg in segments:
            color = STATE_COLORS.get(seg['state'], '#DDDDDD')
            axh.axvspan(seg['start_time'], seg['end_time'], alpha=0.08, color=color)
        # 顶点标注（失重点 = 最大高度）
        axh.annotate(f"顶点 ≈{alt['max_h']:.1f}m",
                     xy=(alt['apex_time'], alt['max_h']),
                     xytext=(alt['apex_time'] - alt['rise_time'] * 0.9,
                             alt['max_h'] * 0.85),
                     arrowprops=dict(arrowstyle='->', color='#8E44AD'),
                     fontsize=10, color='#8E44AD', fontweight='bold')
        # 起飞/失重竖线
        axh.axvline(x=alt['launch_time'], color='red', linestyle='--',
                    linewidth=1, alpha=0.7, label='起飞')
        axh.axvline(x=alt['apex_time'], color='orange', linestyle='--',
                    linewidth=1, alpha=0.7, label='失重(顶点)')
        axh.set_ylabel('估算高度 (m)')
        axh.legend(loc='upper left', fontsize=8)
        axh.grid(True, alpha=0.3)
        axh.set_title('高度估算 (起飞→失重区间积分，仅供参考)', fontsize=9, alpha=0.7)

    # ---------- 子图3: 三轴加速度 ----------
    ax2 = axes[2] if alt else axes[1]
    ax2.plot(times, ax_g, color='#E74C3C', linewidth=0.8, label='Ax')
    ax2.plot(times, ay_g, color='#27AE60', linewidth=0.8, label='Ay')
    ax2.plot(times, az_g, color='#2980B9', linewidth=0.8, label='Az')
    ax2.axhline(y=0, color='gray', linewidth=0.3)
    ax2.set_ylabel('三轴加速度 (g)')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ---------- 子图4: 三轴角速度 ----------
    ax3 = axes[3] if alt else axes[2]
    ax3.plot(times, gx_dps, color='#E74C3C', linewidth=0.8, label='Gx')
    ax3.plot(times, gy_dps, color='#27AE60', linewidth=0.8, label='Gy')
    ax3.plot(times, gz_dps, color='#2980B9', linewidth=0.8, label='Gz')
    ax3.axhline(y=0, color='gray', linewidth=0.3)
    ax3.set_ylabel('角速度 (deg/s)')
    ax3.set_xlabel('时间 (s, 相对起飞)')
    ax3.legend(loc='upper right', fontsize=8)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ============================================================
# 主入口
# ============================================================
def main():
    if len(sys.argv) < 2:
        # 没有参数，尝试找当前目录下的 .bin 文件
        bin_files = [f for f in os.listdir('.') if f.lower().endswith('.bin')]
        if bin_files:
            print(f"未指定文件，发现当前目录下的 .bin 文件:")
            for i, f in enumerate(bin_files):
                print(f"  [{i}] {f}")
            try:
                choice = int(input(f"\n选择文件编号 (0-{len(bin_files)-1}): "))
                filepath = bin_files[choice]
            except (ValueError, IndexError):
                print("无效选择")
                sys.exit(1)
        else:
            print("用法: python flight_analyzer.py <数据文件.bin>")
            print("  或将 .bin 文件放在同一目录下直接运行")
            sys.exit(1)
    else:
        filepath = sys.argv[1]

    if not os.path.exists(filepath):
        print(f"[错误] 文件不存在: {filepath}")
        sys.exit(1)

    print(f"读取文件: {filepath}")
    print(f"文件大小: {os.path.getsize(filepath)} 字节\n")

    entries = parse_file(filepath)
    if not entries:
        sys.exit(1)

    segments = analyze(entries)
    alt = estimate_altitude(entries)

    if has_matplotlib:
        plot_data(entries, segments, alt)
    else:
        print("\n[提示] 安装 matplotlib 后可查看图表: pip install matplotlib")


if __name__ == '__main__':
    main()
