import sys
import os
import subprocess
import cv2
import serial
import numpy as np
import time

# シリアルポートの設定
uart = serial.Serial('/dev/ttyS1', 57600, timeout=0.5)

# v4l2-ctlコマンドの実行
try:
    print("v4l2-ctlコマンドを実行します")
    subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl', 'exposure_auto=1'], check=True)
    print("オートゲインをオフにしました")
    subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl', 'white_balance_temperature_auto=0'], check=True)
    print("オートホワイトバランスをオフにしました")
except subprocess.CalledProcessError as e:
    print(f"v4l2-ctlコマンドの実行中にエラーが発生しました: {e}")
    sys.exit(1)

# パケット生成
class Packet:
    def __init__(self, command=0):
        self.b = np.zeros(6, dtype=np.uint8)
        self.makePacket(command)

    def makePacket(self, command=0):
        self.b[0] = 0xff
        self.b[1] = 0x55
        low = np.uint8(command & 0xff)
        high = np.uint8((command >> 8) & 0xff)
        self.b[2] = low
        self.b[3] = ~low & 0xff
        self.b[4] = high
        self.b[5] = ~high & 0xff
    
    def to_bytes(self):
        return bytes(self.b)

# ボタン
NONE = np.uint16(0x0000)  # 何も押さない
BU = np.uint16(0x0001)    # Uボタン
BD = np.uint16(0x0002)    # Dボタン
BL = np.uint16(0x0004)    # Lボタン
BR = np.uint16(0x0008)    # Rボタン
B1 = np.uint16(0x0010)    # 1ボタン
B2 = np.uint16(0x0020)    # 2ボタン
B3 = np.uint16(0x0040)    # 3ボタン
B4 = np.uint16(0x0080)    # 4ボタン
B5 = np.uint16(0x0100)    # 5ボタン
B6 = np.uint16(0x0200)    # 6ボタン

# モーション
CommandNone = Packet(0)
Forward = Packet(BU | B6)
ForwardSmall = Packet(BU)
Backward = Packet(BD | B6)
BackwardSmall = Packet(BD)
StepRight = Packet(BR)
StepRightSmall = Packet(BR | B4)
TurnRight = Packet(BR | B5)
TurnRightSmall = Packet(BR | B5 | B4)
StepLeft = Packet(BL)
StepLeftSmall = Packet(BL | B4)
TurnLeft = Packet(BL | B5)
TurnLeftSmall = Packet(BL | B5 | B4)
KickRight = Packet(B2)
KickLeft = Packet(B1)
KeeperRight = Packet(BR | B1 | B5)
KeeperCenter = Packet(BL | B1 | B5 | B6)
KeeperLeft = Packet(BL | B1 | B5)
StandUp = Packet(BU | B1)
TorqueOn = Packet(BU | B3 | B5 | B6)
TorqueOff = Packet(BD | B3 | B5 | B6)
AroundRight = Packet(BL | B1)
AroundLeft = Packet(BR | B1)
Forwardsteerleft = Packet(BU | BL)
Forwardsteerright = Packet(BU | BR)
Backwardsteerleft = Packet(BD | BL)
Backwardsteerright = Packet(BD | BR)
SidekickRight = Packet(B3)
SidekickLeft = Packet(B4)

# モーション決定
class MotionTask:
    def __init__(self, uart):
        self.uart = uart
        self.command_tmp = None

    def send_motion(self, x, y):
        x_left = 120
        x_center_left = 260
        x_center = 310
        x_center_right = 360
        x_right = 500
        y_top = 250
        y_bottom = 380

        if x <= x_center_left and y <= y_top:
            command = TurnLeftSmall
            print("TurnLeftSmall")
        elif x_center_left < x <= x_center_right and y <= y_top:
            command = Forward
            print("Forward")
        elif x_center_right < x and y <= y_top:
            command = TurnRightSmall
            print("TurnRightSmall")
        elif x <= x_left and y_top < y:
            command = StepLeft
            print("StepLeft")
        elif x_left < x <= x_center_left and y_top < y:
            command = StepLeftSmall
            print("StepLeftSmall")
        elif x_center_left < x <= x_center and y_bottom < y:
            command = KickLeft
            print("KickLeft")
        elif x_center_left < x <= x_center_right and y_top < y <= y_bottom:
            command = ForwardSmall
            print("ForwardSmall")
        elif x_center < x <= x_center_right and y_bottom < y:
            command = KickRight
            print("KickRight")
        elif x_center_right < x <= x_right and y_top < y:
            command = StepRightSmall
            print("StepRightSmall")
        elif x_right < x and y_top < y:
            command = StepRight
            print("StepRight")
        else:
            command = CommandNone
            print("CommandNone")
            self.command_tmp = command
        # モーション送出
        self.uart.write(command.to_bytes())
        print("モーションに対応するボタン番号を送りました")

# ボール検出
class ColorTracker:
    def __init__(self, motiontask):
        # LAB色空間での閾値
        self.l_min, self.l_max = 120, 250
        self.a_min, self.a_max = 120, 160
        self.b_min, self.b_max = 180, 230
        
        self.capture = cv2.VideoCapture(0)
        # オートカメラ設定オフ (python直実行だと使えない)
        #self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        #self.capture.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.motiontask = motiontask

    def process(self, image):
        # LAB色空間に変換
        image_lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
        
        # 指定した色範囲でマスク作成（白黒）
        mask = cv2.inRange(
            image_lab,
            np.array([self.l_min, self.a_min, self.b_min]),
            np.array([self.l_max, self.a_max, self.b_max])
        )
        
        # モルフォロジー演算でノイズ除去
        # -モルフォロジー演算用のカーネル（フィルタ）を作成
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # -オープニング処理（収縮→膨張）で小さなノイズを除去
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 輪郭検出
        contours, hierarchy = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )[-2:]

        # 輪郭が検出されたか確認
        if len(contours) == 0:
            print("輪郭が検出されませんでした")
            return None, None, None, mask
    
        # 最大の輪郭を見つける
        max_contour = max(contours, key=cv2.contourArea)
        
        # 外接円を計算
        (x, y), radius = cv2.minEnclosingCircle(max_contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        # 重心を計算
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            mass_center = (cx, cy)
        else:
            mass_center = center
            
        return center, radius, mass_center, mask

    def run(self):
        log_dir = "/media/sdcard/log"
        os.makedirs(log_dir, exist_ok=True)  # ディレクトリが存在しない場合は作成

        # 連番のファイル名を作成
        i = 0
        while True:
            log_file_path = os.path.join(log_dir, f"log_{i:03d}.txt") # ファイル名を3桁の数字でフォーマット
            if not os.path.exists(log_file_path):
                break
            i += 1
        
        with open(log_file_path, "w") as f:
            while True:
                ret, frame = self.capture.read()
                if not ret:
                    print("カメラから画像を取得できませんでした")
                    break
                
                center, radius, mass_center, mask = self.process(frame)
                if center is None:
                    print("ボールが検出できませんでした")
                    continue
                
                # 処理結果出力
                print(f"外接円の中心座標: (x={center[0]}, y={center[1]})")
                print(f"重心座標: (x={mass_center[0]}, y={mass_center[1]})")
                print(f"輪郭の半径: {radius}ピクセル")

                # MotionTaskにボールの中心点座標を渡す
                x, y = center
                self.motiontask.send_motion(x, y)

                # ログファイルに書き込み
                output_log = f"{center[0]}, {center[1]}, {mass_center[0]}, {mass_center[1]}, {radius}\n"
                f.write(output_log)
            
        self.capture.release()

def cleanup():
    print("\nクリーンアップを実行します...")
    try:
        if 'tracker' in globals() and tracker.capture.isOpened():
            tracker.capture.release()
        if 'uart' in globals() and uart.is_open:
            uart.close()
    except Exception as e:
        print(f"クリーンアップ中にエラーが発生しました: {e}")
    print("終了処理が完了しました")

if __name__ == "__main__":
    try:
        motiontask = MotionTask(uart)
        tracker = ColorTracker(motiontask)
        print("OpenCV バージョン: " + cv2.__version__)
        print("Start")
        tracker.run()
    except KeyboardInterrupt:
        print("\nプログラムが中断されました")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        cleanup()