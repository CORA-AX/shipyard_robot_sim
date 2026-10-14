# Go2 강화학습

학습 코드와 실행 결과는 프로젝트의 `rl/` 아래에서 관리한다.

```text
rl/
├── go2/
│   ├── tasks/flat_tracking/   # Gym 등록, 환경·PPO 설정, 보상, 명령 샘플러
│   ├── train.py              # Isaac Lab 학습 실행기
│   ├── play.py               # 학습 정책 재생
│   ├── evaluate.py           # 고정 명령의 추종·발 접촉 평가
│   ├── analyze.py            # 실행 폴더의 TensorBoard 기록 분석
│   └── gait_metrics.py       # 기록된 보행 주기·동기화·미끄러짐 측정
└── rsl_rl/                   # 로그·모델·저장 설정·평가 결과 (Git 제외)
    ├── go2_flat_tracking/
    └── unitree_go2_flat/
```

`scenarios/`는 viewer 장면과 로봇 배치, `control/go2/`는 정책 추론과 제어 연결을 담당한다.
제어에 선택한 정책 복사본은 `assets/go2/policies/`에 둔다.
외부 Isaac Lab와 RSL-RL 라이브러리는 기존 `third_party/`와 Conda 환경에서 사용한다.

## 환경 활성화

프로젝트 루트에서 실행한다.

```bash
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
```

## 처음부터 학습

현재 v4는 대각선 쌍 `FL–RR`, `FR–RL`의 동기화·교대와 속도 추종, 높이 목표 0.28m를 학습한다.
접지 중 미끄러짐을 벌하고, 움직이는 명령에서 발을 0.4초 넘게 들면 비용이 증가한다.
0.6초 이후에는 착지할 때까지 발 하나당 dt 적용 전 -2.0이 계속 적용된다.
정상적인 짧은 스윙은 벌하지 않으며 trot 보상 0.5는 시작부터 적용한다.
관측 48개·액션 12개로 현재 Go2 제어기와 호환된다. 설정은 초기 실험값이며 보행 개선을 보장하지 않는다.

```bash
CUDA_VISIBLE_DEVICES=0,1,2,3 \
python -m torch.distributed.run --standalone --nproc_per_node=4 \
  rl/go2/train.py \
  --task Shipyard-Go2-Flat-Tracking-v0 \
  --distributed --num_envs 1024 --max_iterations 5000 --seed 42 \
  --run_name trot_v4_fresh --headless
```

`--num_envs`는 GPU당 환경 수이므로 총 4096개다. 처음부터 학습할 때는 `--resume`,
`--load_run`, `--checkpoint`를 생략한다. 결과는
`rl/rsl_rl/go2_flat_tracking/<실행시각>_trot_v4_fresh/`에 저장된다.
5000회 실행의 마지막 모델은 `model_4999.pt`다. 분산 실행은 한 폴더를 공유하고,
대표 설정과 로그는 rank 0, 다른 GPU 설정은 `params/ranks/rank_<번호>/`에 저장한다.

프로젝트 루트에서는 `python -m rl.go2.train ...`도 지원한다.
멀티 GPU에서는 위 명령의 `rl/go2/train.py`를 사용하거나
`python -m torch.distributed.run --standalone --nproc_per_node=4 --module rl.go2.train ...`을 사용한다.

## 추가학습과 재생

체크포인트를 재개할 때 `--load_run`에는 `rl/rsl_rl/go2_flat_tracking/` 아래의 폴더명,
`--checkpoint`에는 해당 폴더 안의 모델 파일명을 지정한다.

```bash
python -m rl.go2.train --task Shipyard-Go2-Flat-Tracking-v0 \
  --num_envs 1024 --max_iterations 1000 --seed 42 \
  --resume --load_run 2026-10-13_00-34-53_tracking_4gpu \
  --checkpoint model_10000.pt --run_name resume_example --headless

python -m rl.go2.play --task Shipyard-Go2-Flat-Tracking-Play-v0 \
  --num_envs 16 \
  --checkpoint rl/rsl_rl/go2_flat_tracking/2026-10-13_00-34-53_tracking_4gpu/model_10000.pt
```

`--max_iterations`는 재개 후 추가 횟수다. 모델과 optimizer는 체크포인트에서 읽고 보상은
현재 task 설정을 사용한다. PLAY는 noise와 외력을 끄지만 다른 학습 무작위화는 남는다.

## 로그 분석과 보행 평가

```bash
tensorboard --logdir rl/rsl_rl/go2_flat_tracking --host 127.0.0.1 --port 6006

python -m rl.go2.analyze \
  --run rl/rsl_rl/go2_flat_tracking/2026-10-13_00-34-53_tracking_4gpu

python -m rl.go2.evaluate \
  --checkpoint assets/go2/policies/tracking_v2_10000.pt \
  --case forward_04 --num_envs 16 --steps 1600 --seed 42 \
  --output rl/rsl_rl/go2_gait_eval/baseline_forward_04.json \
  --device cuda:0 --headless --kit_args=--/renderer/multiGpu/enabled=false
```

`analyze`는 어떤 길이의 학습·재개 실행도 처리하며 `<run>/analysis/`에
`training_overview.json`, `scalar_history.json`, `training_overview.png`를 생성한다.
보행 평가는 전진 `forward_02/04/08`, 정지 `stand`, 회전 `yaw_05/yaw_neg_05`,
전진 회전 `forward_turn`, 후진 `backward_04`, 횡이동 `lateral_04`를 지원한다.
동일 이름의 JSON 통계·NPZ 발 기록·PNG 접촉 타임라인을 저장한다.
기본 32초 중 첫 2초와 리셋 후 2초는 상태 평균에서 제외하지만 넘어짐은 전체 구간에서 센다.
발 기록은 50Hz이며 더 짧은 접촉을 놓칠 수 있다. 같은 초기 조건의 16환경은 독립 seed 16개가 아니다.

모델 선택에는 총 reward 외에 발별 접지 비율·대각선 교대·완전한 보행 주기·미끄러짐·추종·넘어짐을
함께 사용한다. 정지에서도 대각선 접촉 일치율이 높을 수 있으므로 이 지표 하나로 trot을 판정하지 않는다.

## 코드와 이전 결과

- 보상 가중치, 높이, PPO: `go2/tasks/flat_tracking/env_cfg.py`
- 보상 연산: `go2/tasks/flat_tracking/mdp.py`
- 명령 분포: `go2/tasks/flat_tracking/commands.py`
- Gym task 등록: `go2/tasks/flat_tracking/__init__.py`

기존 루트 `rsl_rl/`의 모든 파일은 `rl/rsl_rl/`로 이동했다. 체크포인트와 이전 결과 내용은
보존했으며, 이전 로그·저장 설정에 들어 있는 당시의 절대 경로와 Python 모듈명은 역사 기록으로
남아 있다. 새 실행은 새 경로와 모듈명을 기록한다. 이전 실행 폴더의 실험 전용 분석 스크립트 대신
위 공통 `analyze`와 `evaluate`를 사용한다.

```bash
python -m unittest discover -s tests -p 'test_go2_*.py' -v
```

2026-10-14 정리 후 검증: 기존 800개 파일(체크포인트 534개)의 해시 일치,
Go2 테스트 33개, 네 실행기의 모듈 실행/도움말, 짧은 신규·재개 로그 분석을 확인했다.
옮긴 `model_10000.pt`에서 4 GPU × 환경 8개 × 2회 및 단일 GPU × 환경 16개 × 2회
재개를 검증했다. 분산 실행의 폴더/로그는 각각 하나였고 모델과 로그 값은 유한값이었다.
상세 기록은 [정리 검증 결과](reorganization_validation.json)에 있다. 본 학습은 시작하지 않았다.
