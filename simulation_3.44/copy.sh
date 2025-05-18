#!/bin/bash

# Ensure OUTPUT_DIR is defined
if [ -z "$OUTPUT_DIR" ]; then
  echo "❌ ERROR: OUTPUT_DIR not set"
  exit 1
fi

for file in ${CMAKE_CURRENT_SOURCE_DIR}/*.copy.*; do
  [ -f "$file" ] || continue
  base=$(basename "$file")
  link_name=$(echo "$base" | sed 's/\.copy\.//')
  echo "🔗 Creating symlink $OUTPUT_DIR/$link_name → $file"
  ln -sf "$file" "$OUTPUT_DIR/$link_name"
done
