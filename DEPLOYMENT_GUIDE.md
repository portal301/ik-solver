# GitHub 배포 가이드

## 1. 로컬 Git 저장소 초기화

```bash
cd c:\dev\ikfast-generator\ikfast-robotics

# Git 초기화
git init

# 모든 파일 추가
git add .

# 첫 커밋
git commit -m "Initial commit: IKFast Robotics unified library"
```

## 2. GitHub Repository 생성

### GitHub에서:

1. 회사 GitHub 계정에 로그인
2. "New repository" 클릭
3. Repository 이름: `ikfast-robotics`
4. Description: "Unified IKFast solver library for multiple robot models"
5. Public/Private 선택
6. **중요**: "Add a README file" 체크하지 않기 (이미 있음)
7. "Create repository" 클릭

## 3. 로컬 → GitHub 연결

```bash
# GitHub repository URL로 원격 저장소 추가
git remote add origin https://github.com/YOUR-COMPANY/ikfast-robotics.git

# 기본 브랜치 이름 설정
git branch -M main

# Push
git push -u origin main
```

## 4. Release 생성

### GitHub에서:

1. Repository 페이지 → "Releases" → "Create a new release"
2. Tag: `v1.0.0`
3. Release title: `v1.0.0 - Initial Release`
4. Description:
   ```markdown
   ## IKFast Robotics Library v1.0.0

   첫 번째 공식 릴리즈입니다.

   ### 지원 로봇
   - Kawasaki KJ125
   - Yaskawa GP4 (진행 중)
   - 추가 로봇 모델 개발 중

   ### 주요 기능
   - 통합 IK 솔버 API
   - Forward kinematics 검증
   - 여러 솔루션 탐색
   - C++ 예제 코드 포함

   ### 설치 방법
   README.md 참고
   ```
5. "Publish release" 클릭

## 5. 팀원 공유

### 방법 1: Git Clone

팀원들에게 다음 명령어 공유:

```bash
git clone https://github.com/YOUR-COMPANY/ikfast-robotics.git
cd ikfast-robotics
mkdir build && cd build
cmake ..
make
```

### 방법 2: Release 다운로드

GitHub Release 페이지에서 Source code (zip) 다운로드

### 방법 3: 바이너리 배포 (선택사항)

빌드된 라이브러리 파일을 릴리즈에 첨부:

```bash
# 빌드
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make

# 패키징
tar -czf ikfast-robotics-v1.0.0-linux-x64.tar.gz \
    libikfast_robotics.so \
    ../include/ikfast_solver.h \
    ../README.md
```

GitHub Release에 `ikfast-robotics-v1.0.0-linux-x64.tar.gz` 업로드

## 6. 문서화

### GitHub Pages 활성화 (선택사항)

1. Repository → Settings → Pages
2. Source: "main" branch, "docs" folder
3. README.md가 자동으로 홈페이지로 표시됨

### Wiki 생성 (선택사항)

1. Repository → Wiki → "Create the first page"
2. 상세 사용법, API 레퍼런스 등 추가

## 7. 지속적 통합 (CI/CD)

### GitHub Actions 설정

`.github/workflows/ci.yml` 파일 생성:

```yaml
name: Build and Test

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v2

    - name: Install dependencies
      run: |
        sudo apt-get update
        sudo apt-get install -y cmake g++ liblapack-dev libblas-dev

    - name: Build
      run: |
        mkdir build && cd build
        cmake ..
        make

    - name: Test
      run: |
        cd build
        ctest --output-on-failure
```

## 8. 팀원 초대

1. Repository → Settings → Collaborators
2. "Add people" 클릭
3. 팀원 GitHub 사용자명 입력
4. 권한 선택 (Read, Write, Admin)

## 9. README 배지 추가 (선택사항)

README.md 상단에 추가:

```markdown
[![Build Status](https://github.com/YOUR-COMPANY/ikfast-robotics/workflows/Build%20and%20Test/badge.svg)](https://github.com/YOUR-COMPANY/ikfast-robotics/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
```

## 10. 체크리스트

배포 전 확인사항:

- [ ] 모든 파일이 커밋되었는가?
- [ ] README.md에 회사 정보 업데이트
- [ ] LICENSE 파일 확인
- [ ] 예제 코드 테스트 완료
- [ ] .gitignore 설정 확인
- [ ] 민감한 정보(비밀번호 등) 제거
- [ ] 문서 링크 정상 작동 확인

## 사용 예시

팀원이 라이브러리를 사용하는 전체 과정:

```bash
# 1. Clone
git clone https://github.com/YOUR-COMPANY/ikfast-robotics.git
cd ikfast-robotics

# 2. Build
mkdir build && cd build
cmake ..
make

# 3. Run example
./simple_ik_example

# 4. Use in their project
# my_project/CMakeLists.txt:
find_package(ikfast_robotics REQUIRED)
target_link_libraries(my_target ikfast_robotics::ikfast_robotics)
```

## 문제 해결

### "Permission denied" 오류

```bash
# SSH 키 설정
ssh-keygen -t ed25519 -C "your.email@company.com"
cat ~/.ssh/id_ed25519.pub
# GitHub → Settings → SSH keys에 추가
git remote set-url origin git@github.com:YOUR-COMPANY/ikfast-robotics.git
```

### 큰 파일 Push 실패

```bash
# Git LFS 사용
git lfs install
git lfs track "*.so"
git lfs track "*.dll"
git add .gitattributes
git commit -m "Add Git LFS"
```

## 다음 단계

1. 로봇 모델 추가 (GP4, GP7 등)
2. Python 바인딩 개발
3. ROS 패키지 통합
4. 성능 벤치마크 추가
