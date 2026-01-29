## FASTDDS DISCOVERY SETTING ##
1. 설치
sudo apt update
sudo apt install fastdds-tools

2. 설정 (서버) - 로봇, 로봇별 설정(P05L 기준)
 1) mkdir -p /etc/fastdds
 2) sudo nano discovery-server.conf
    작성
    SERVER_ID=1
    SERVER_IP=192.168.0.105
    SERVER_PORT=11811

 3) sudo nano /etc/systemd/system/fastdds-discovery-server.service
    작성
    [Unit]
    Description=Fast DDS Discovery Server
    After=network-online.target
    Wants=network-online.target

    [Service]
    Type=simple
    EnvironmentFile=/etc/fastdds/discovery-server.conf

    ExecStart=/usr/bin/fastdds discovery \
    --server-id ${SERVER_ID} \
    --ip-address ${SERVER_IP} \
    --port ${SERVER_PORT} \
    --backup

    Restart=always
    RestartSec=1

    StandardOutput=journal
    StandardError=journal

    [Install]
    WantedBy=multi-user.target

 4) sudo systemctl daemon-reload
    sudo systemctl enable --now fastdds-discovery-server.service

 5) bashrc
    export ROS_DOMAIN_ID=20
    export ROS_DISCOVERY_SERVER=";192.168.0.105:11811"

    * 지금은 SERVER_ID가 1이라서 ; -> 0이면 ; 없이 -> 2면 ;; 2개

 6) 확인
   systemctl status fastdds-discovery-server.service --no-pager : acitavt 되고 있는지
   sudo journalctl -u fastdds-discovery-server.service -n 50 --no-pager : 안될때 에러 로그 확인

 7) 재시작
   sudo systemctl daemon-reload
   sudo systemctl reset-failed fastdds-discovery-server.service
   sudo systemctl restart fastdds-discovery-server.service

3. PC
 $bashrc
 export ROS_DOMAIN_ID=20 
 export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
 export ROS_DISCOVERY_SERVER="192.168.0.103:11811;192.168.0.105:11811;192.168.0.106:11811" 
 export ROS_SUPER_CLIENT=TRUE