#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def main() -> None:
    p = argparse.ArgumentParser(description="选择机型参数并激活")
    p.add_argument("--machine", required=True, help="机型ID，如 M003")
    p.add_argument("--root", default=".", help="工程根目录")
    args = p.parse_args()

    root = Path(args.root).resolve()
    src = root / "config" / "machines" / f"{args.machine}.yaml"
    dst = root / "config" / "machine.active.yaml"

    if not src.exists():
        raise SystemExit(f"[ERROR] 机型配置不存在: {src}")

    shutil.copyfile(src, dst)
    txt = src.read_text(encoding="utf-8")

    print(f"[OK] 已激活机型: {args.machine}")
    print(f"[OK] active file: {dst}")
    for key in ["host:", "endpoint:", "db_address:"]:
        for line in txt.splitlines():
            if key in line:
                print("  "+line.strip())


if __name__ == "__main__":
    main()
