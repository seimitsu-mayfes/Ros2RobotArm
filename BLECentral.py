import asyncio
import json
import socket
from bleak import BleakClient, BleakScanner
import aioconsole
from threading import Thread

# ESP32のBLEデバイス名
DEVICE_NAME = "ESP32 BLE Device"  # 接続するデバイスの名前

# UUIDs
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  # サービスのUUID
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # キャラクタリスティックのUUID

# TCP/IP設定
TCP_IP = '127.0.0.1'
TCP_PORT = 5000

class BLEController:
    def __init__(self):
        self.client = None
        self.connected = False
        self.ble_device = None
        self.debug_mode = True  # デバッグモードを有効化

    async def connect_ble(self):
        """BLEデバイスに接続"""
        if self.debug_mode:
            print("[DEBUG] Skipping BLE connection in debug mode")
            self.connected = True
            return True

        self.ble_device = await BleakScanner.find_device_by_name(DEVICE_NAME)
        if not self.ble_device:
            print(f"Could not find device with name '{DEVICE_NAME}'")
            return False

        self.client = BleakClient(self.ble_device)
        try:
            await self.client.connect()
            print(f"Connected to {self.ble_device.name}")
            self.connected = True
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    async def send_angles(self, angles):
        """サーボモータの角度をBLEデバイスに送信"""
        if not self.connected:
            return

        try:
            # デバッグ情報を表示
            print("\n[DEBUG] Received angles (radians):", angles)
            degrees = [int(angle * 180 / 3.14159) for angle in angles]
            print("[DEBUG] Converted to degrees:", degrees)

            if not self.debug_mode:
                # 実際のBLE送信(デバッグモードでは実行しない)
                for i, angle in enumerate(angles, start=1):
                    degrees = int(angle * 180 / 3.14159)
                    command = f"{i} {degrees}"
                    await self.client.write_gatt_char(CHARACTERISTIC_UUID, command.encode('utf-8'))
                    print(f"Sent command: {command}")
                    await asyncio.sleep(0.1)
        except Exception as e:
            print(f"Failed to send angles: {e}")

    async def disconnect(self):
        """BLEデバイスから切断"""
        if self.client and not self.debug_mode:
            await self.client.disconnect()
        self.connected = False
        print("[DEBUG] Disconnected from BLE device")

class TCPClient:
    def __init__(self, ble_controller):
        self.sock = None
        self.ble_controller = ble_controller
        self.running = True

    def connect(self):
        """TCP/IPサーバーに接続"""
        retry_count = 0
        max_retries = 5
        base_delay = 1  # 基本待機時間(秒)

        while self.running:
            try:
                if retry_count >= max_retries:
                    print("[DEBUG] Maximum retry attempts reached. Waiting for 30 seconds before resetting...")
                    asyncio.get_event_loop().run_until_complete(asyncio.sleep(30))
                    retry_count = 0
                    continue

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # キープアライブオプションを設定
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                # 接続タイムアウトを設定
                self.sock.settimeout(5)
                
                self.sock.connect((TCP_IP, TCP_PORT))
                print("[DEBUG] Connected to TCP server")
                retry_count = 0  # 接続成功したらカウントをリセット
                self.receive_data()
                
            except ConnectionRefusedError:
                retry_count += 1
                delay = min(base_delay * (2 ** retry_count), 30)  # 指数バックオフ(最大30秒)
                print(f"[DEBUG] Failed to connect to TCP server. Retry {retry_count}/{max_retries}. Waiting {delay} seconds...")
                asyncio.get_event_loop().run_until_complete(asyncio.sleep(delay))
                
            except Exception as e:
                retry_count += 1
                delay = min(base_delay * (2 ** retry_count), 30)
                print(f"[DEBUG] TCP error: {e}. Retry {retry_count}/{max_retries}. Waiting {delay} seconds...")
                asyncio.get_event_loop().run_until_complete(asyncio.sleep(delay))
                
            finally:
                if self.sock:
                    self.sock.close()

    def receive_data(self):
        """サーバーからデータを受信"""
        buffer = ""
        while self.running:
            try:
                data = self.sock.recv(1024).decode('utf-8')
                if not data:
                    print("[DEBUG] Server disconnected")
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        print("[DEBUG] Received raw data:", line)
                        json_data = json.loads(line)
                        if 'angles' in json_data:
                            # BLEデバイスに角度データを送信
                            asyncio.get_event_loop().run_until_complete(
                                self.ble_controller.send_angles(json_data['angles'])
                            )
                    except json.JSONDecodeError as e:
                        print(f"[DEBUG] JSON decode error: {e}")

            except Exception as e:
                print(f"[DEBUG] Error receiving data: {e}")
                break

    def stop(self):
        """クライアントを停止"""
        self.running = False
        if self.sock:
            self.sock.close()

async def main():
    """メイン関数"""
    print("[DEBUG] Starting BLECentral.py in debug mode")
    
    # BLEコントローラーを初期化
    ble_controller = BLEController()
    
    # デバッグモードではBLE接続をスキップ
    if not await ble_controller.connect_ble():
        return

    # TCP/IPクライアントを初期化と接続スレッド開始
    tcp_client = TCPClient(ble_controller)
    tcp_thread = Thread(target=tcp_client.connect)
    tcp_thread.start()

    try:
        # メインループ
        while True:
            # キー入力待機
            command = await aioconsole.ainput("Press 'q' to quit: ")
            if command.lower() == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        # クリーンアップ
        print("[DEBUG] Cleaning up...")
        tcp_client.stop()
        await ble_controller.disconnect()
        tcp_thread.join()

if __name__ == "__main__":
    asyncio.run(main())
