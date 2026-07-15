# Q2 Gap-Follow — F1TENTH Roboracer 2026

Q2 자율주행 (정적 장애물 2개 회피 + 3바퀴 완주) 패키지.

## 핵심 동작
- **Wall-Follow** (오른쪽 벽 PID 추종) — 평상시
- **Gap-Follow** (Disparity Extender + Safety Bubble) — 장애물 감지 시 자동 전환
- **모드 전환**: LiDAR로 0.5m 클러스터 감지 → GAP / 1.5초 이상 없음 → WALL 복귀

## 의존성
- ROS 2 Foxy
- f1tenth_gym_ros 시뮬레이터

## 빌드 & 실행
```bash
colcon build --packages-select q2_gap_follow --symlink-install
ros2 launch q2_gap_follow q2_launch.py
```

## 안정 셋팅 (v1)
- max_speed: 3.0 m/s
- 한쪽 벽 추종 (right wall, 1.0m target)
- 장애물 3m 전에 감지, 0.35m 안전 마진
- Spielberg_q2 맵에서 3바퀴 완주 검증

## 팀
YAX / Yonsei University

## 시연 영상

https://github.com/user-attachments/assets/d2fbce14-d385-494a-8bb1-a2a96ca663ba

3.0 m/s 안정 셋팅으로 Spielberg_q2 맵 3바퀴 완주. wall-follow (벽 추종) → 장애물 감지 → gap-follow (회피) → wall-follow 복귀가 자동으로 일어남.
