#!/bin/bash

# 삭제할 디렉토리 경로
TARGET_DIR="/home/jmseo1204/catkin_ws/build/git_fast_livo2"

# 디렉토리 존재 확인
if [ -d "$TARGET_DIR" ]; then
  echo "Deleting all files in: $TARGET_DIR"
  rm -rf "$TARGET_DIR"/*
  rm -rf "$TARGET_DIR"/.[!.]* "$TARGET_DIR"/..?*
  echo "Done."
else
  echo "Directory not found: $TARGET_DIR"
  exit 1
fi