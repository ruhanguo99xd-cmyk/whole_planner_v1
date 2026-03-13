# DEVELOPMENT_WORKFLOW

这份文档约定本仓库后续的日常开发方式，尽量保证：

- `master` 保持稳定
- 日常改动在独立工作区完成
- 关键节点有备份分支可回退

## 当前分支

- `master`
  - 稳定主线
  - 当前已推送到 GitHub
- `workspace/20260313-integrated-mission-planner-dev`
  - 当前推荐开发分支
  - 对应独立工作区：
    - `/home/ruhanguo/shovel_robot/whole_planner_v1_dev_workspace`
- `backup/20260313-integrated-mission-planner`
  - 关键快照分支
  - 只做留档，不建议继续日常开发

## 当前远端

- `origin`
  - GitHub 正式远端
  - `https://github.com/ruhanguo99xd-cmyk/whole_planner_v1`
- `backup-local`
  - 本地 bare 备份仓库
  - `/home/ruhanguo/shovel_robot/whole_planner_v1_backup.git`

## 推荐工作方式

### 1. 日常开发

进入独立开发工作区：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1_dev_workspace
git status
git pull
```

开发完成后：

```bash
git add -A
git commit -m "feat: your change"
git push
```

说明：
- 这个工作区已经跟踪 `origin/workspace/20260313-integrated-mission-planner-dev`
- 所以日常不需要每次写完整远端名

### 2. 合回 master

当开发分支已经过验证，准备合回主线时：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
git pull
git merge workspace/20260313-integrated-mission-planner-dev
git push
```

如果你想先看差异：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
git log --oneline --left-right master...workspace/20260313-integrated-mission-planner-dev
git diff --stat master...workspace/20260313-integrated-mission-planner-dev
```

### 3. 再做一次快照备份

适合用于：
- 一个阶段完成
- 一轮联调通过
- 准备做较大重构前

示例：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
git switch master
git pull
git switch -c backup/20260313-some-milestone
git push origin backup/20260313-some-milestone
git push backup-local backup/20260313-some-milestone
```

## 推荐提交粒度

尽量按功能切 commit，不要把很多独立事情揉在一起。

推荐拆法：

- `feat: ...`
- `fix: ...`
- `refactor: ...`
- `test: ...`
- `docs: ...`

例如：

```bash
git commit -m "feat: add point-pick goal interaction to mobility hmi"
git commit -m "feat: publish optimization candidate path for excavation hmi"
git commit -m "docs: update runbook and interface notes for hmi"
```

## 提交前最小检查

至少跑下面这些：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1_dev_workspace
python3 -m py_compile src/mission_operator_hmi/mission_operator_hmi/helpers.py src/mission_operator_hmi/mission_operator_hmi/integrated_operator_hmi.py
PYTHONPATH=src/mission_operator_hmi:$PYTHONPATH python3 -m pytest -q src/mission_operator_hmi/test/test_helpers.py
bash scripts/build_phase2_minimal.sh
```

如果改了调度或规划主链，再补：

```bash
colcon --log-base log_phase2 test --build-base build_phase2 --install-base install_phase2 --packages-select mission_dispatcher plc_adapter mobility_planner_core excavation_planner_core mission_operator_hmi
colcon --log-base log_phase2 test-result --all --verbose
```

## 避免的操作

- 不要直接在 `backup/...` 分支上长期开发
- 不要把未验证的大改动直接推到 `master`
- 不要在两个 worktree 里同时改同一个分支

## 常用命令速查

看工作区：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
git worktree list
```

看远端：

```bash
git remote -v
```

看当前分支关系：

```bash
git branch -vv
```

推主线：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
git push origin master
```

推开发分支：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1_dev_workspace
git push
```
