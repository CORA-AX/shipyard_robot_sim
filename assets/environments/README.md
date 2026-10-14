# 공통 환경 자산

모든 로봇이 공유할 배경 USD·지도·재질·텍스처의 위치다.
배치와 실행 설정은 루트 `scenarios/`에서 관리한다.
기본 `warehouse`는 NVIDIA Simple Warehouse의 Isaac Sim 5.0 USD를 사용한다.
자산 출처는 `warehouse/sources.lock.json`, 장면 설정은 `scenarios/warehouse/`에서 관리한다.
로봇 USD·URDF·메시는 기존 `assets/go2`, `assets/hunter2`에 보관한다.
