#!/usr/bin/env bash
# generate-docs-skeleton.sh
# 扫描 ~/Apollo 源码结构，在 docs/ 下生成对应的文档骨架
# 每个源码目录 → 一个 index.md（含标题和源码路径提示）
# 子目录中有 .cc/.h 文件才会生成文档

set -euo pipefail

APOLLO_ROOT="$HOME/Apollo"
DOCS_DIR="$(cd "$(dirname "$0")" && pwd)/docs"

# 统计目录中 .cc + .h 文件数
count_sources() {
  find "$1" -name "*.cc" -o -name "*.h" 2>/dev/null | wc -l
}

# 将 snake_case 目录名转为可读标题
to_title() {
  echo "$1" | sed 's/_/ /g' | sed 's/\b\(.\)/\u\1/g'
}

# 生成单个 index.md
# $1: 文档目录路径  $2: 标题  $3: 源码路径（相对 Apollo 根）  $4: order (可选)
write_index() {
  local doc_dir="$1" title="$2" src_path="$3" order="${4:-}"
  mkdir -p "$doc_dir"
  local file="$doc_dir/index.md"
  [ -f "$file" ] && return  # 不覆盖已有文件

  local header="---\ntitle: \"$title\"\n"
  [ -n "$order" ] && header+="order: $order\n"
  header+="---\n\n# $title\n\n"
  header+="> 源码路径: \`$src_path\`\n\n"
  header+="<!-- TODO: 填充内容 -->\n"

  echo -e "$header" > "$file"
}

# ─── 1. Guide ───
mkdir -p "$DOCS_DIR/guide"
write_index "$DOCS_DIR/guide" "指南" "" 1

for topic in getting-started architecture build config data-flow dev; do
  title=$(to_title "$topic")
  file="$DOCS_DIR/guide/$topic.md"
  [ -f "$file" ] && continue
  cat > "$file" <<EOF
---
title: "$title"
---

# $title

<!-- TODO: 填充内容 -->
EOF
done

# ─── 2. Cyber ───
write_index "$DOCS_DIR/cyber" "CyberRT 框架" "cyber/" 1

# cyber 子模块：只为有源码的目录生成
for dir in "$APOLLO_ROOT"/cyber/*/; do
  [ ! -d "$dir" ] && continue
  name=$(basename "$dir")
  # 跳过非代码目录
  [[ "$name" =~ ^(conf|docs|doxy-docs|examples|proto|python|tools)$ ]] && continue
  src_count=$(count_sources "$dir")
  [ "$src_count" -eq 0 ] && continue
  title=$(to_title "$name")
  write_index "$DOCS_DIR/cyber/$name" "$title" "cyber/$name/" ""
done

# ─── 3. Modules ───
write_index "$DOCS_DIR/modules" "模块总览" "modules/" 1

for mod_dir in "$APOLLO_ROOT"/modules/*/; do
  [ ! -d "$mod_dir" ] && continue
  mod_name=$(basename "$mod_dir")
  # 跳过纯消息包和非代码目录
  [[ "$mod_name" =~ ^(common_msgs|safety_manager_msgs|statistics_msgs|BUILD)$ ]] && continue
  src_count=$(count_sources "$mod_dir")
  [ "$src_count" -eq 0 ] && continue

  # 文档目录名：下划线转连字符
  doc_name=$(echo "$mod_name" | tr '_' '-')
  mod_title=$(to_title "$mod_name")
  write_index "$DOCS_DIR/modules/$doc_name" "$mod_title" "modules/$mod_name/" ""

  # 大模块（>50 源文件）：展开子目录
  if [ "$src_count" -gt 50 ]; then
    for sub_dir in "$mod_dir"/*/; do
      [ ! -d "$sub_dir" ] && continue
      sub_name=$(basename "$sub_dir")
      [[ "$sub_name" =~ ^(proto|conf|data|launch|dag|BUILD)$ ]] && continue
      sub_count=$(count_sources "$sub_dir")
      [ "$sub_count" -eq 0 ] && continue
      sub_doc_name=$(echo "$sub_name" | tr '_' '-')
      sub_title=$(to_title "$sub_name")
      write_index "$DOCS_DIR/modules/$doc_name/$sub_doc_name" "$sub_title" "modules/$mod_name/$sub_name/" ""

      # planning/tasks 和 planning/scenarios：再展开一层
      if [[ "$mod_name" == "planning" && ("$sub_name" == "tasks" || "$sub_name" == "scenarios" || "$sub_name" == "planners") ]]; then
        for leaf_dir in "$sub_dir"/*/; do
          [ ! -d "$leaf_dir" ] && continue
          leaf_name=$(basename "$leaf_dir")
          leaf_count=$(count_sources "$leaf_dir")
          [ "$leaf_count" -eq 0 ] && continue
          leaf_doc_name=$(echo "$leaf_name" | tr '_' '-')
          leaf_title=$(to_title "$leaf_name")
          # 这些写成单文件而非子目录
          local_file="$DOCS_DIR/modules/$doc_name/$sub_doc_name/$leaf_doc_name.md"
          [ -f "$local_file" ] && continue
          cat > "$local_file" <<EOF
---
title: "$leaf_title"
---

# $leaf_title

> 源码路径: \`modules/$mod_name/$sub_name/$leaf_name/\`

<!-- TODO: 填充内容 -->
EOF
        done
      fi
    done
  fi
done

# ─── 4. Reference ───
write_index "$DOCS_DIR/reference" "参考" "" ""

# ─── 统计 ───
total=$(find "$DOCS_DIR" -name "*.md" | wc -l)
echo "✅ 骨架生成完成：共 $total 个 md 文件"
echo ""
echo "目录结构："
find "$DOCS_DIR" -name "*.md" | sed "s|$DOCS_DIR/||" | sort | head -60
echo "..."
