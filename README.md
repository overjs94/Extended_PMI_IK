# Extended PMI-IK

URDF → symbolic FK/Jacobian → constrained midpoint IK → measured benchmarks → 3D playback.

기존 PMI-IK RAL 코드와 maximal-coordinate white note를 바탕으로 구성한 **연구용 참조 구현**입니다. 기존 ROS 실행 파일의 재현 또는 논문의 병렬 가속 결과로 간주하지 않습니다.

## 실행

Python **3.12**, Node **24**, C compiler (C 함수 검증용)가 필요합니다. 기존 체크아웃에서 실행하세요. 클라우드 작업마다 이미 격리되므로 별도 Git worktree를 만들지 않습니다.

```bash
bash scripts/setup.sh
bash scripts/start.sh
```

서버는 8000 포트에서 UI와 API를 함께 제공합니다. 개발 시에는 API를 실행한 상태에서 `npm --prefix web run dev`를 사용합니다.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q
.venv/bin/extended-pmi generate models/panda.urdf --output generated/panda
gcc -O2 -shared -fPIC generated/panda/kinematics.c -o generated/panda/kinematics.so -lm
OPENBLAS_NUM_THREADS=1 .venv/bin/extended-pmi benchmark \
  --scenario dual --seeds 7 11 19 --output results/dual.json
```

현재는 source checkout + editable install을 지원합니다. 모델과 웹 리소스까지 포함한 독립 wheel 배포는 구성하지 않았습니다.

## 구성

| 경로 | 역할 |
|---|---|
| `src/extended_pmi_ik/kinematics.py` | URDF 파싱, 고정 링크 병합, CasADi FK/Jacobian 및 C 코드 생성 |
| `constraints.py` | maximal body twist에 대한 구조·grasp·joint limit·거리 제약의 자동 미분 |
| `solver.py`, `backends.py`, `splitting.py` | reduced/maximal PMI, OSQP/qpOASES, VI active set, Schur, ADMM, DRS |
| `guidance.py` | 장애물 우회를 위한 receding geometric preview; 완전한 MPC는 아님 |
| `web/` | Three.js 뷰어, convergence/energy plot, playback, 실제 측정값 CSV/JSON |
| `models/registry.json` | 공개 로봇의 출처·고정 commit·목표 링크·고정 접촉 링크 |
| `models/external/` | 다운로드한 외부 모델 캐시, 출처·checksum·license 파일; Git에서 제외 |
| `reference/pmi_ik/` | 제공된 원본 파일 중 핵심 코드의 무수정 스냅샷과 SHA256 목록 |
| `ros/extended_pmi_ik_ros/` | 선택적 ROS 1 bridge; 현재 환경에는 ROS가 없어 통합 미검증 |

## 로봇과 시각화

공개 humanoid 7종을 대상으로 합니다: **Unitree G1, Unitree H1, Booster T1, Fourier GR1 T1, Fourier N1, ToddlerBot, ergoCub**. Berkeley biped도 별도로 포함합니다. 이는 IK 벤치마크 카탈로그이며 SOTA 성능 순위는 아닙니다.

로봇팔은 제공된 **Franka Panda**, **KUKA iiwa 7/14**, **UR5/UR10**입니다. 다른 시스템으로 **KUKA–Allegro 23 DoF**, **Allegro 16 DoF**, **ANYmal C**, 그리고 **2·4·8개 Panda의 합성 공동 조작**을 포함합니다. 합성 humanoid 운반 예제도 별도로 유지합니다. 모델별 실제 검증 상태는 `docs/model-validation.json`을 확인하세요.

- **Collision geometry**: URDF `<collision>`의 geometry/origin을 표시합니다.
- **Visual meshes**: URDF `<visual>`의 geometry/origin과 원본 material을 표시합니다. Panda는 제공된 DAE, 외부 모델은 원본 STL/DAE/OBJ를 사용합니다.
- 일부 공급자의 visual/collision mesh가 동일할 수 있습니다. 실제 상세도는 원본 자산에 따릅니다.
- 메시 표시와 충돌 계산은 별개입니다. 장애물 예제의 충돌 계산은 명시된 구형 프록시에 한정합니다. 메시 기반 전신·자가 충돌 계산은 아직 없습니다.
- 외부 캐시에는 원본 URDF, 변환 URDF, `provenance.json`, 가능한 upstream license가 보존됩니다. 재배포 조건은 원 출처에 따릅니다.

## 해법

| formulation | 중앙집중 | body-block Schur | consensus ADMM | product-space DRS |
|---|---|---|---|---|
| Reduced QP-PMI | OSQP, qpOASES | — | — | — |
| Maximal QP-PMI (`C=0`) | OSQP, qpOASES | 지원 | 지원 | 지원 |
| Maximal Skew-PMI (`Cᵀ=-C`) | VI active set | 지원 | 일반 QP ADMM 적용 불가 | 지원 |

qpOASES는 CasADi의 **실제 qpOASES plugin**, OSQP는 공식 Python binding을 사용합니다. Skew-PMI를 `DᵀD`로 바꾸거나 skew 부분을 버려 QP로 위장하지 않습니다. 분할법의 body resolvent와 factor projection은 병렬화 가능한 독립 batch로 구현했으나, 현재 실행·검증은 NumPy CPU입니다. GPU/분산 worker 실행 또는 speedup은 검증하지 않았습니다.

측정은 동일 seed/초기 상태/목표를 사용합니다. 모델·constraint 함수 생성 시간은 setup 시간으로, IK 반복·FK·제약 평가·frame 기록은 solve wall time으로 구분합니다. UI는 한 seed의 탐색 결과이며 통계적 논문 비교에는 CLI 다중 seed와 동일 장비·thread 설정이 필요합니다. `converged`는 pose/constraint tolerance 충족이며 KKT 또는 정지 속도의 인증은 아닙니다.

## 연구 방향과 남은 통합

[설계 및 receding-horizon 논의](docs/research-design.md), [원본 코드 대응](docs/source-provenance.md)을 참고하세요. 다른 ChatGPT 프로젝트의 `extended_pmi_ik_draft` 대화와 contact-contraction 저장소에는 이 세션에서 접근할 수 없으므로, 그 내용까지 반영했다고 주장하지 않습니다.

원본 ROS 빌드는 `ik_msgs`, ROS 1, NLopt, FCL, 외부 qpOASES/qpSWIFT tree에 의존합니다. 제공된 ZIP에는 해당 third-party 전체가 없었습니다. 새 코어는 이를 요구하지 않습니다. ROS bridge를 사용하려면 rospy와 코어를 같은 Python 환경에 설치하고 `/robot_description`, `/joint_states`, `~target_link`, `~world_frame`을 설정하세요. `~target` (`PoseStamped`)을 받아 `~solution` (`JointState`)만 발행하며 하드웨어 명령은 발행하지 않습니다.
