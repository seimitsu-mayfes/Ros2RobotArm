# Copyright 2020-2023 Tiryoh<tiryoh@gmail.com>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# This Dockerfile is based on https://github.com/AtsushiSaito/docker-ubuntu-sweb
# which is released under the Apache-2.0 license.

# ベースイメージとして Ubuntu 22.04 (Jammy) を使用
FROM ubuntu:jammy-20240808

# ビルド時のターゲットプラットフォーム引数
ARG TARGETPLATFORM
LABEL maintainer="Tiryoh<tiryoh@gmail.com>"

# シェルをbashに設定
SHELL ["/bin/bash", "-c"]

#=========================================
# OSアップグレード
# システムパッケージを最新の状態に更新
#=========================================
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# デスクトップ環境のインストール
# Ubuntu MATE デスクトップ環境をセットアップ
#=========================================
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ubuntu-mate-desktop && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# 必要なパッケージのインストール
# VNC、開発ツール、その他の必要なユーティリティをインストール
#=========================================
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git sudo python3-pip tini \
        build-essential vim sudo lsb-release locales \
        bash-completion tzdata terminator \
        dos2unix && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# VNCブラウザアクセス設定
# ブラウザベースのVNCクライアント(noVNC)とWebSocket実装をセットアップ
#=========================================
RUN git clone https://github.com/AtsushiSaito/noVNC.git -b add_clipboard_support /usr/lib/novnc
RUN pip install git+https://github.com/novnc/websockify.git@v0.10.0
RUN ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

#=========================================
# VNC表示設定
# リモートリサイズ機能をデフォルトで有効化
#=========================================
RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

#=========================================
# システム設定の最適化
# 自動更新とクラッシュレポートを無効化してコンテナの動作を安定化
#=========================================
RUN sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades
RUN sed -i 's/enabled=1/enabled=0/g' /etc/default/apport

#=========================================
# Firefoxブラウザのインストール
# Mozilla Firefoxの最新バージョンをインストール
#=========================================
RUN DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:mozillateam/ppa -y && \
    echo 'Package: *' > /etc/apt/preferences.d/mozilla-firefox && \
    echo 'Pin: release o=LP-PPA-mozillateam' >> /etc/apt/preferences.d/mozilla-firefox && \
    echo 'Pin-Priority: 1001' >> /etc/apt/preferences.d/mozilla-firefox && \
    apt-get update -q && \
    apt-get install -y \
    firefox && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# VSCodiumのインストール
# オープンソースのコードエディタVSCodiumをインストール
#=========================================
RUN wget https://gitlab.com/paulcarroty/vscodium-deb-rpm-repo/raw/master/pub.gpg \
    -O /usr/share/keyrings/vscodium-archive-keyring.asc && \
    echo 'deb [ signed-by=/usr/share/keyrings/vscodium-archive-keyring.asc ] https://paulcarroty.gitlab.io/vscodium-deb-rpm-repo/debs vscodium main' \
    | tee /etc/apt/sources.list.d/vscodium.list && \
    apt-get update -q && \
    apt-get install -y codium && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# ROS2のインストール
# ROS2 Humbleディストリビューションと関連ツールをセットアップ
#=========================================
ENV ROS_DISTRO humble
# desktop or ros-base
ARG INSTALL_PACKAGE=desktop

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update

#=========================================
# MoveIt2のインストール
# ROS2 Humbleに対応したMoveIt2パッケージをインストール
#=========================================
RUN apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-moveit && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# シミュレーションパッケージのインストール
# amd64アーキテクチャの場合のみGazeboとその他のシミュレーションツールをインストール
#=========================================
# Not ready for arm64 for now (July 28th, 2020)
# https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56#issuecomment-1196359860
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
    apt-get update -q && \
    apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-ign && \
    rm -rf /var/lib/apt/lists/*; \
    fi

#=========================================
# apt-get補完の有効化
# コンテナ内でのapt-getコマンド補完を有効化
#=========================================
RUN rm /etc/apt/apt.conf.d/docker-clean

COPY ./entrypoint.sh /
RUN dos2unix /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-c", "/entrypoint.sh" ]

#=========================================
# デフォルトユーザー設定
# コンテナのデフォルトユーザー名とパスワードを設定
#=========================================
ENV USER ubuntu
ENV PASSWD ubuntu
