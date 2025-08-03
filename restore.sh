#!/bin/bash

RESTORE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SOURCE_DIR=~/catkin_ws/src/git_fast_livo2


# 1. 소스 디렉토리 존재 여부 확인
if [ ! -d "$SOURCE_DIR" ]; then
    echo "오류: 복원할 소스 디렉토리를 찾을 수 없습니다."
    echo "경로: $SOURCE_DIR"
    exit 1
fi

# Git 프로젝트 디렉토리로 이동
cd "$RESTORE_DIR"

# Git이 추적하는 파일 목록을 기반으로, SOURCE_DIR에서 현재 디렉토리로 파일들을 복사(덮어쓰기)
# git ls-files: 현재 프로젝트에서 Git이 추적하는 파일 목록만 가져옵니다.
# rsync: 해당 목록을 기반으로 SOURCE_DIR에서 RESTORE_DIR(./)로 파일을 가져옵니다.
git ls-files -z | rsync -av --files-from=- --from0 "$SOURCE_DIR/" ./

echo "복원이 완료되었습니다."