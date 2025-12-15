# SLAM Toolbox

▶ 개요

ROS2 기반 2D SLAM 패키지

로봇의 위치 추정(Localization) 과 지도 생성(Mapping) 을 동시에 수행

▶ 입력 센서

2D LiDAR (/scan)

Odometry (/odom)

TF 프레임 (odom → base → laser)

▶ 주요 기능

실시간 지도 생성 및 갱신

Graph-based SLAM 알고리즘 적용

지도 저장 / 재사용 / 온라인 최적화 지원

Nav2와 완전 호환

▶ 사용 목적

자율주행 로봇의 환경 인식

실내 공간 맵 생성 및 위치 추적

내비게이션 및 경로 계획의 기반 데이터 제공

▶ 기존 방식 대비 장점

ROS2 공식 지원

gmapping 대비 지도 품질 및 안정성 향상

실시간·장기 운용에 적합

▶ 한 줄 요약

slam_toolbox는 ROS2 자율주행 로봇을 위한
실시간 2D 지도 생성 및 위치 추정 핵심 SLAM 엔진이다.
