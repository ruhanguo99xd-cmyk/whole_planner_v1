#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

apply=0
if [[ "${1:-}" == "--apply" ]]; then
  apply=1
fi

keep_dirs=(
  "./build"
  "./install"
  "./log"
  "./log_install_fix"
  "./.git/logs"
)

candidates=()
while IFS= read -r path; do
  skip=0
  for keep in "${keep_dirs[@]}"; do
    if [[ "$path" == "$keep" ]]; then
      skip=1
      break
    fi
  done
  if [[ $skip -eq 0 ]]; then
    candidates+=("$path")
  fi
done < <(find . -maxdepth 1 \( -type d -name 'build*' -o -type d -name 'install*' -o -type d -name 'log*' \) | sort)

if [[ ${#candidates[@]} -eq 0 ]]; then
  echo "No legacy build/install/log directories to clean."
  exit 0
fi

echo "Legacy artifact directories:"
for path in "${candidates[@]}"; do
  echo "  $path"
done

if [[ $apply -ne 1 ]]; then
  echo
  echo "Dry run only. To archive these directories safely, run:"
  echo "  bash scripts/clean_legacy_artifacts.sh --apply"
  exit 0
fi

archive_root="./.cleanup_archive/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$archive_root"

for path in "${candidates[@]}"; do
  target="$archive_root/$(basename "$path")"
  echo "Archiving $path -> $target"
  mv "$path" "$target"
done

echo
echo "Archive complete."
echo "Archived directories are stored under: $archive_root"
