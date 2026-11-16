# 저장소 구성 안내

LIMO 자율주행 대회(Autorace 2026)용 ROS2 워크스페이스(`wego_ws`) 코드 저장소입니다.

## 버전 관리 방식: 저장소 1개 + 버전별 브랜치

서로 다른 코드 버전을 **하나의 저장소(rlagnltn/limo_ai)에 브랜치로 나누어** 관리합니다.

```
rlagnltn/limo_ai
├─ main   ← README만 유지
├─ v1     ← 버전 1 (LIMO의 /home/wego/wego_ws)
└─ v2     ← 버전 2 (추후 업로드)
```

이 방식을 택한 이유:
- 세 버전 모두 같은 ROS2 워크스페이스 구조라 브랜치 간 비교(diff)·병합이 쉽다.
- 협업자 권한은 저장소 단위이므로 한 번 추가하면 모든 버전을 편집할 수 있다.
- `git checkout <브랜치>`만으로 버전을 바꿔 빌드할 수 있다.

### 나머지 버전 업로드 방법

```bash
# 저장소를 받은 뒤 새 브랜치 생성
git clone https://github.com/rlagnltn/limo_ai.git wego_ws_v2 && cd wego_ws_v2
git checkout -b v2 origin/v1
# 해당 버전의 src/ 등 파일로 교체한 뒤
git add -A
git commit -m "버전 2 업로드"
git push -u origin v2
```

## 업로드 제외 항목 (`.gitignore`)

| 항목 | 제외 이유 |
|---|---|
| `build/`, `install/`, `log/` | `colcon build` 시 재생성되는 빌드 산출물 |
| `drive_bags/` | 주행 기록 bag 데이터, 대용량 |
| `*.zip` | 압축 파일 (예: `perception_bundle_*.zip`, `made_dataset.zip`) |
| `src/dl_ros2_application/weight/made_dataset/` | 학습용 원본 데이터셋 이미지, 대용량 |
| `__pycache__/`, `*.pyc` | Python 캐시 |

모델 가중치(`*.pt`, `*.onnx`, `*.engine`, `*.h5`)는 주행에 필요하므로 포함합니다.

## 외부 패키지 처리

`src/navigation2`, `src/OrbbecSDK_ROS2`는 원래 외부 GitHub 저장소에서 받은 코드이지만,
로컬 수정 사항을 함께 보관하기 위해 **서브모듈이 아닌 일반 폴더로 포함**했습니다.

- navigation2 원본: https://github.com/ros-navigation/navigation2
- OrbbecSDK_ROS2 원본: https://github.com/orbbec/OrbbecSDK_ROS2

## 빌드

```bash
cd wego_ws
colcon build
source install/setup.bash
```
