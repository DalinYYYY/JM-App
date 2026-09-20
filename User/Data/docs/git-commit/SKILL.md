---
name: "git-commit"
description: "使用 Conventional Commits 规范创建标准化的语义化 git 提交。当用户要求提交、创建 git commit 或提到 /commit 时调用, 需要使用中文生成"
---

# Git Commit 规范化提交（Conventional Commits）

## 概述

使用 Conventional Commits 规范创建标准化的、语义化的 git 提交。通过分析实际 diff 来确定合适的 type、scope 和 commit message。

## Conventional Commit 格式

```
<type>[可选 scope]: <描述>

[可选 body]

[可选 footer(s)]
```

## 提交类型（Commit Types）

| Type       | 用途                           |
| ---------- | ------------------------------ |
| `feat`     | 新功能                         |
| `fix`      | Bug 修复                       |
| `docs`     | 仅文档变更                     |
| `style`    | 格式/样式调整（不涉及逻辑）    |
| `refactor` | 代码重构（无新功能/无修复）    |
| `perf`     | 性能优化                       |
| `test`     | 新增/更新测试                  |
| `build`    | 构建系统/依赖变更              |
| `ci`       | CI 配置变更                    |
| `chore`    | 维护/杂项                      |
| `revert`   | 回滚提交                       |

## 破坏性变更（Breaking Changes）

```
# 在 type/scope 后加感叹号
feat!: remove deprecated endpoint

# 使用 BREAKING CHANGE footer
feat: allow config to extend other configs

BREAKING CHANGE: `extends` key behavior changed
```

## 工作流程

### 1. 分析 Diff

```bash
# 如果有文件已暂存，查看暂存区的 diff
git diff --staged

# 如果没有暂存内容，查看工作区的 diff
git diff

# 同时检查状态
git status --porcelain
```

### 2. 暂存文件（如需要）

如果没有已暂存内容，或希望按不同方式分组：

```bash
# 暂存指定文件
git add path/to/file1 path/to/file2

# 按模式暂存
git add *.test.*
git add src/components/*

# 交互式暂存
git add -p
```

**严禁提交密钥**（.env、credentials.json、私钥等）。

### 3. 生成提交信息

分析 diff 后确定：

- **Type**：这是什么类型的变更？
- **Scope**：影响哪个区域/模块？
- **Description**：一句话总结改了什么（现在时、祈使语气、<72 字符）

### 4. 执行提交

```bash
# 单行提交
git commit -m "<type>[scope]: <description>"

# 多行提交（带 body/footer）
git commit -m "$(cat <<'EOF'
<type>[scope]: <description>

<可选 body>

<可选 footer>
EOF
)"
```

## 最佳实践

- 每次提交只包含一个逻辑变更
- 使用现在时："add" 而非 "added"
- 使用祈使语气："fix bug" 而非 "fixes bug"
- 关联 issue：`Closes #123`、`Refs #456`
- 描述保持在 72 字符以内

## Git 安全协议

- 严禁修改 git config
- 严禁未经明确请求执行破坏性命令（--force、hard reset）
- 除非用户要求，严禁跳过 hooks（--no-verify）
- 严禁强制推送到 main/master
- 如果提交因 hooks 失败，修复后创建新提交（不要 amend）
