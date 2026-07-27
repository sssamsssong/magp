# logs/

`stanley_controller.py`가 **주행 종료 시 1회** 저장하는 실주행 기록. 팀 전체 공유용
(개인 대화 기록인 `chat_log/`와 다르게 gitignore 안 됨 — git으로 커밋해서 공유).

## 폴더 구조

```
logs/<YYYYmmdd_HHMMSS>_<mode>/
  ├── gains.yaml        그 주행 시점의 race_script.yaml 전체 스냅샷
  │                     (게인 6종 + speed 섹션 + map_name/waypoints 필드 포함)
  └── driven_path.csv   실주행 경로: t(초), x, y, speed  (0.1m 이상 이동 시마다 기록)
```

웨이포인트(레퍼런스 경로) 원본은 복사하지 않는다 — `gains.yaml`의 `map_name`/
`waypoints` 필드가 `waypoints/<waypoints>.csv`를 가리키므로, 그 파일과
`driven_path.csv`를 같이 읽으면 비교 가능하다 (웨이포인트 파일 자체가 이후에
재생성되어 바뀔 수 있다는 점은 감안할 것).

## 저장 시점 / 안전성

50Hz 제어 루프 안에서 매 포인트를 디스크에 쓰지 않는다 — 메모리에만 쌓아두고
(`self._trail_log`, 최대 3000점), 노드 종료(`Ctrl+C`/모드 전환 시 재시작) 시
`save_run_log()`에서 한 번에 파일로 flush한다. 즉 조향 제어 타이밍에는 영향 없음.

## 사용 예 — 게인 비교

두 실행의 `gains.yaml`을 diff해서 뭐가 바뀌었는지 보고, 각 `driven_path.csv`를
그때 쓴 웨이포인트 CSV와 비교해 코너별 CTE(횡오차)를 계산하면, 스크린샷으로
눈대중하는 것보다 정확하게 게인 변경 전후를 비교할 수 있다.

의미 있는 결과만 골라 커밋할 것 — 테스트마다 자동으로 쌓이니 지저분한 실패
기록까지 전부 푸시하지는 말 것.
