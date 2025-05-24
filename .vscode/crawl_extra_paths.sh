#!/bin/bash

# カレントディレクトリ (VSCode の ${workspaceFolder} に相当)
WORKSPACE_FOLDER=$(pwd)
BASE_PATH="$WORKSPACE_FOLDER/.venv/lib/python3.10/site-packages/isaacsim"
ENV_PATH="\${workspaceFolder}/.venv/lib/python3.10/site-packages/isaacsim"

# ベースディレクトリが存在するか確認
if [ ! -d "$BASE_PATH" ]; then
  echo "Error: $BASE_PATH が存在しません。"
  exit 1
fi

# isaacsim 内のサブディレクトリを再帰的にクロールして処理
for sub_dir in "$BASE_PATH"/*; do
  if [ -d "$sub_dir" ]; then
    sub_dir_name=$(basename "$sub_dir")
    for folder in "$sub_dir"/*; do
      if [ -d "$folder" ]; then
        folder_name=$(basename "$folder")
        echo "\"${ENV_PATH}/$sub_dir_name/$folder_name\","
      fi
    done
  fi
done
