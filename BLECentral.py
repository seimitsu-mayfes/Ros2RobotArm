import asyncio
from bleak import BleakClient, BleakScanner
import aioconsole

# ESP32のBLEデバイス名
DEVICE_NAME = "ESP32 BLE Device"  # 接続するデバイスの名前

# UUIDs
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  # サービスのUUID
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # キャラクタリスティックのUUID

async def notification_handler(sender, data):
    """通知を処理するコールバック関数"""
    print(f"Received notification: {data.decode()}")  # 受信した通知をデコードして表示

async def user_input(client):
    """ユーザーからの入力を処理し、BLEデバイスにコマンドを送信する関数"""
    while True:
        # ユーザーからのコマンド入力を非同期で待機
        command = await aioconsole.ainput("Enter command (e.g., '1 80'): ")
        if command.lower() == 'q':
            return False  # 'q'が入力されたらループを終了
        elif command:
            # コマンドをUTF-8でエンコードしてBLEデバイスに送信
            await client.write_gatt_char(CHARACTERISTIC_UUID, command.encode('utf-8'))
            print(f"Sent command: {command}")  # 送信したコマンドを表示
        else:
            print("Invalid command. Please enter a valid command or 'q' to quit.")  # 無効なコマンドの場合のメッセージ

async def main():
    """メイン関数：BLEデバイスに接続し、通知を受信する"""
    # デバイス名でBLEデバイスをスキャンし、デバイスを見つける
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    
    if not device:
        print(f"Could not find device with name '{DEVICE_NAME}'")  # デバイスが見つからない場合のメッセージ
        return

    # デバイスに接続し、接続が確立されたら以下の処理を行う
    async with BleakClient(device) as client:
        print(f"Connected to {device.name}")  # デバイスに接続したことを表示

        # 通知を受信するためのハンドラを設定
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        try:
            # ユーザー入力を処理してコマンドを送信
            await user_input(client)
        except asyncio.CancelledError:
            pass

        # 通知の受信を停止
        await client.stop_notify(CHARACTERISTIC_UUID)

    print("Disconnected")  # デバイスから切断したことを表示

# メイン関数を実行
asyncio.run(main())
