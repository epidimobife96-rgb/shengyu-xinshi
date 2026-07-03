import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import wave
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import sounddevice as sd
except ImportError:
    sd = None


# =========================
# 基本参数
# =========================
FS_DEFAULT = 44100

FSK_FREQS = {
    "00": 1500,
    "01": 2500,
    "10": 3500,
    "11": 4500,
}

DIGIT_SYMBOLS = {
    "0": ["00", "01"],
    "1": ["00", "10"],
    "2": ["00", "11"],
    "3": ["01", "00"],
    "4": ["01", "10"],
    "5": ["01", "11"],
    "6": ["10", "00"],
    "7": ["10", "01"],
    "8": ["10", "11"],
    "9": ["11", "00"],
}

DIGIT_PREAMBLE = ["00", "01", "10", "11"]
DIGIT_START = ["01", "10"]
DIGIT_END = ["10", "11"]
DIGIT_GAP_MS = 70
DIGIT_REPEAT = 3


class FSK4AudioTester:
    def __init__(self, root):
        self.root = root
        self.root.title("4-FSK 声波测试信号发生器")

        self.current_audio = np.array([], dtype=np.float32)
        self.current_fs = FS_DEFAULT

        self.build_ui()
        self.generate_single_tone(1500)

    def build_ui(self):
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(control_frame, text="采样率 Fs").pack(anchor="w")
        self.fs_var = tk.StringVar(value=str(FS_DEFAULT))
        ttk.Entry(control_frame, textvariable=self.fs_var, width=12).pack(anchor="w", pady=2)

        ttk.Label(control_frame, text="幅度 0~1，建议 0.1~0.3").pack(anchor="w")
        self.amp_var = tk.StringVar(value="0.25")
        ttk.Entry(control_frame, textvariable=self.amp_var, width=12).pack(anchor="w", pady=2)

        ttk.Label(control_frame, text="单频持续时间 / s").pack(anchor="w")
        self.duration_var = tk.StringVar(value="2.0")
        ttk.Entry(control_frame, textvariable=self.duration_var, width=12).pack(anchor="w", pady=2)

        ttk.Label(control_frame, text="4-FSK 单码元时间 / ms").pack(anchor="w")
        self.symbol_ms_var = tk.StringVar(value="220")
        ttk.Entry(control_frame, textvariable=self.symbol_ms_var, width=12).pack(anchor="w", pady=2)

        ttk.Separator(control_frame).pack(fill=tk.X, pady=8)

        ttk.Label(control_frame, text="单频测试").pack(anchor="w")

        for bits, freq in FSK_FREQS.items():
            ttk.Button(
                control_frame,
                text=f"{bits} → {freq} Hz",
                command=lambda f=freq: self.generate_single_tone(f)
            ).pack(fill=tk.X, pady=2)

        ttk.Separator(control_frame).pack(fill=tk.X, pady=8)

        ttk.Button(
            control_frame,
            text="生成 00 01 10 11 循环测试",
            command=self.generate_fsk_loop
        ).pack(fill=tk.X, pady=2)

        ttk.Label(control_frame, text="一位数字测试").pack(anchor="w", pady=(8, 2))

        digit_frame = ttk.Frame(control_frame)
        digit_frame.pack(fill=tk.X, pady=2)

        for i in range(10):
            ttk.Button(
                digit_frame,
                text=str(i),
                width=4,
                command=lambda d=str(i): self.generate_digit(d)
            ).grid(row=i // 5, column=i % 5, padx=1, pady=1)

        ttk.Button(
            control_frame,
            text="生成前导码 + HELLO 测试帧",
            command=self.generate_hello_frame
        ).pack(fill=tk.X, pady=2)

        ttk.Separator(control_frame).pack(fill=tk.X, pady=8)

        ttk.Button(control_frame, text="播放", command=self.play_audio).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="停止", command=self.stop_audio).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="保存 WAV", command=self.save_wav).pack(fill=tk.X, pady=2)

        ttk.Separator(control_frame).pack(fill=tk.X, pady=8)

        self.info_var = tk.StringVar(value="未播放")
        ttk.Label(control_frame, textvariable=self.info_var, wraplength=180).pack(anchor="w", pady=5)

        self.fig, (self.ax_time, self.ax_fft) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.tight_layout(pad=3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def get_params(self):
        try:
            fs = int(float(self.fs_var.get()))
            amp = float(self.amp_var.get())
            duration = float(self.duration_var.get())
            symbol_ms = float(self.symbol_ms_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "请检查 Fs、幅度、时间是否为数字")
            return None

        if not (0 < amp <= 1):
            messagebox.showerror("参数错误", "幅度建议在 0~1 之间")
            return None

        return fs, amp, duration, symbol_ms

    @staticmethod
    def fade_edges(y, fs, fade_ms=5):
        """给声音首尾加淡入淡出，避免扬声器啪声。"""
        n = int(fs * fade_ms / 1000)
        if n <= 0 or len(y) < 2 * n:
            return y

        fade_in = np.linspace(0, 1, n)
        fade_out = np.linspace(1, 0, n)

        y[:n] *= fade_in
        y[-n:] *= fade_out
        return y

    def make_tone(self, freq, duration, fs, amp):
        t = np.arange(int(duration * fs)) / fs
        y = amp * np.sin(2 * np.pi * freq * t)
        return self.fade_edges(y.astype(np.float32), fs)

    def make_silence(self, duration, fs):
        return np.zeros(int(duration * fs), dtype=np.float32)

    def make_symbol_audio(self, symbols, symbol_s, fs, amp, gap_ms=0):
        y_all = []
        gap_s = gap_ms / 1000.0

        for index, s in enumerate(symbols):
            freq = FSK_FREQS[s]
            y_all.append(self.make_tone(freq, symbol_s, fs, amp))
            if gap_s > 0 and index != len(symbols) - 1:
                y_all.append(self.make_silence(gap_s, fs))

        return np.concatenate(y_all).astype(np.float32)

    def generate_single_tone(self, freq):
        params = self.get_params()
        if params is None:
            return

        fs, amp, duration, _ = params
        self.current_fs = fs
        self.current_audio = self.make_tone(freq, duration, fs, amp)
        self.info_var.set(f"当前信号：{freq} Hz，{duration:.2f}s，幅度 {amp}")
        self.update_plots(title=f"Single tone: {freq} Hz")

    def generate_fsk_loop(self):
        params = self.get_params()
        if params is None:
            return

        fs, amp, _, symbol_ms = params
        self.current_fs = fs
        ts = symbol_ms / 1000.0

        symbols = ["00", "01", "10", "11"] * 3
        y_all = []

        for s in symbols:
            freq = FSK_FREQS[s]
            y_all.append(self.make_tone(freq, ts, fs, amp))

        self.current_audio = np.concatenate(y_all).astype(np.float32)
        self.info_var.set("当前信号：00 01 10 11 循环 3 次")
        self.update_plots(title="4-FSK loop: 00 01 10 11")

    def generate_digit(self, digit):
        params = self.get_params()
        if params is None:
            return

        fs, amp, _, symbol_ms = params
        self.current_fs = fs
        ts = symbol_ms / 1000.0

        symbols = DIGIT_SYMBOLS[digit]
        y_all = []

        for s in symbols:
            freq = FSK_FREQS[s]
            y_all.append(self.make_tone(freq, ts, fs, amp))

        self.current_audio = np.concatenate(y_all).astype(np.float32)
        self.info_var.set(f"当前信号：数字 {digit}，码元 {' '.join(symbols)}")
        self.update_plots(title=f"Digit {digit}: {' '.join(symbols)}")

    def generate_digit(self, digit):
        params = self.get_params()
        if params is None:
            return

        fs, amp, _, symbol_ms = params
        self.current_fs = fs
        ts = symbol_ms / 1000.0

        data_symbols = DIGIT_SYMBOLS[digit]
        check_symbols = [f"{int(s[0]) ^ 1}{int(s[1]) ^ 1}" for s in data_symbols]
        one_frame = DIGIT_PREAMBLE + DIGIT_START + data_symbols + check_symbols + DIGIT_END
        symbols = []
        for repeat_index in range(DIGIT_REPEAT):
            symbols.extend(one_frame)

        self.current_audio = self.make_symbol_audio(symbols, ts, fs, amp, DIGIT_GAP_MS)
        self.info_var.set(f"Digit {digit} frame x{DIGIT_REPEAT}: {' '.join(one_frame)}")
        self.update_plots(title=f"Digit frame {digit} x{DIGIT_REPEAT}")

    def text_to_bits(self, text):
        data = text.encode("ascii", errors="replace")
        bits = []
        for b in data:
            for i in range(7, -1, -1):
                bits.append((b >> i) & 1)
        return bits

    def bytes_to_bits(self, data):
        bits = []
        for b in data:
            for i in range(7, -1, -1):
                bits.append((b >> i) & 1)
        return bits

    def bits_to_2bit_symbols(self, bits):
        if len(bits) % 2 != 0:
            bits.append(0)

        symbols = []
        for i in range(0, len(bits), 2):
            symbols.append(f"{bits[i]}{bits[i + 1]}")
        return symbols

    def generate_hello_frame(self):
        params = self.get_params()
        if params is None:
            return

        fs, amp, _, symbol_ms = params
        self.current_fs = fs
        ts = symbol_ms / 1000.0

        # 帧格式：55 55 55 55 AA LEN DATA CHECK
        msg = b"HELLO"
        preamble = bytes([0x55, 0x55, 0x55, 0x55])
        start = bytes([0xAA])
        length = bytes([len(msg)])

        checksum_value = 0
        for b in length + msg:
            checksum_value ^= b
        checksum = bytes([checksum_value])

        frame = preamble + start + length + msg + checksum

        bits = self.bytes_to_bits(frame)
        symbols = self.bits_to_2bit_symbols(bits)

        y_all = []
        for s in symbols:
            freq = FSK_FREQS[s]
            y_all.append(self.make_tone(freq, ts, fs, amp))

        self.current_audio = np.concatenate(y_all).astype(np.float32)
        self.info_var.set(
            "当前信号：4-FSK 测试帧\n"
            "55 55 55 55 AA 05 48 45 4C 4C 4F CHECK"
        )
        self.update_plots(title="4-FSK frame: HELLO")

    def update_plots(self, title=""):
        if len(self.current_audio) == 0:
            return

        y = self.current_audio
        fs = self.current_fs

        self.ax_time.clear()
        self.ax_fft.clear()

        # 只显示前 50ms 的波形，便于观察
        show_n = min(len(y), int(0.05 * fs))
        t = np.arange(show_n) / fs * 1000

        self.ax_time.plot(t, y[:show_n])
        self.ax_time.set_title(title + " - waveform")
        self.ax_time.set_xlabel("Time / ms")
        self.ax_time.set_ylabel("Amplitude")
        self.ax_time.grid(True)

        # FFT 频谱
        nfft = min(len(y), fs)
        segment = y[:nfft]

        window = np.hanning(len(segment))
        spectrum = np.fft.rfft(segment * window)
        freqs = np.fft.rfftfreq(len(segment), 1 / fs)

        mag = np.abs(spectrum)
        if np.max(mag) > 0:
            mag_db = 20 * np.log10(mag / np.max(mag) + 1e-12)
        else:
            mag_db = mag

        mask = freqs <= 8000
        self.ax_fft.plot(freqs[mask], mag_db[mask])
        self.ax_fft.set_title("FFT spectrum")
        self.ax_fft.set_xlabel("Frequency / Hz")
        self.ax_fft.set_ylabel("Relative magnitude / dB")
        self.ax_fft.set_ylim(-80, 5)
        self.ax_fft.grid(True)

        for f in FSK_FREQS.values():
            self.ax_fft.axvline(f, linestyle="--", linewidth=0.8)

        self.fig.tight_layout(pad=3)
        self.canvas.draw()

    def play_audio(self):
        if sd is None:
            messagebox.showerror("缺少库", "请先安装 sounddevice：pip install sounddevice")
            return

        if len(self.current_audio) == 0:
            messagebox.showwarning("无音频", "请先生成测试信号")
            return

        sd.stop()
        sd.play(self.current_audio, self.current_fs, blocking=False)
        self.info_var.set(self.info_var.get() + "\n正在播放...")

    def stop_audio(self):
        if sd is not None:
            sd.stop()
        self.info_var.set("已停止播放")

    def save_wav(self):
        if len(self.current_audio) == 0:
            messagebox.showwarning("无音频", "请先生成测试信号")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".wav",
            filetypes=[("WAV files", "*.wav")]
        )

        if not path:
            return

        y = np.clip(self.current_audio, -1, 1)
        y_int16 = (y * 32767).astype(np.int16)

        with wave.open(path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.current_fs)
            wf.writeframes(y_int16.tobytes())

        messagebox.showinfo("已保存", f"已保存到：\n{path}")


if __name__ == "__main__":
    root = tk.Tk()
    app = FSK4AudioTester(root)
    root.mainloop()
