# Ros2RobotArm

ROS2を使用したロボットアームのプロジェクトです。

## 概要

このプロジェクトは、ROS2 Humbleを使用してロボットアームを制御するための環境を提供します。Docker環境を使用することで、簡単にROS2の開発環境を構築できます。

## 環境構成

- ROS2 Humble
- Ubuntu 22.04 (jammy)
- VNC対応デスクトップ環境 (Ubuntu MATE)
- VSCodium
- Firefox

## 必要条件

- Docker
- Docker Compose

## セットアップ方法

1. リポジトリをクローンします:
```bash
git clone [repository-url]
cd Ros2RobotArm
```

2. Docker Composeを使用して環境を起動します:
```bash
docker compose up -d
```

## アクセス方法

### VNCアクセス
- ブラウザで `http://localhost:6080` にアクセス
- デフォルトの解像度: 1024m
- VNCクライアントを使用する場合はポート10000に接続

## プロジェクト構造

```
.
├── docker-compose.yml    # Docker Compose設定ファイル
├── Dockerfile           # Dockerイメージビルド設定
├── entrypoint.sh       # コンテナ起動時の初期化スクリプト
└── ws_colcon/          # ROSワークスペース(ホストとコンテナ間で共有)
```

## 開発環境の特徴

- **VNC対応**: ブラウザベースのVNCクライアントを通じて、グラフィカルな開発環境にアクセス可能
- **VSCodium搭載**: コードの編集や開発作業が可能
- **ボリューム共有**: ホストマシンの`ws_colcon`ディレクトリがコンテナの`/home/ubuntu/ws_colcon`にマウント
- **ポート転送**:
  - 6080: noVNC (ブラウザベースVNC)
  - 10000: VNCサーバー

## 使用方法

1. 環境起動後、ブラウザで `http://localhost:6080` にアクセス
2. Ubuntu MATEデスクトップ環境が表示され、VSCodiumやターミナルを使用して開発可能
3. ROS2のコマンドやツールは通常通り使用可能

## 注意事項

- コンテナ内のユーザー名とパスワードは両方とも `ubuntu` です
- シミュレーションパッケージ(Gazebo)は現在arm64アーキテクチャでは利用できません
