#!/bin/sh
# Builds report.pdf from report.md. Needs pandoc and Google Chrome (no LaTeX).
set -e
cd "$(dirname "$0")"
../.venv/bin/python make_figures.py
pandoc report.md --standalone --embed-resources --css style.css --metadata lang=en -o report.html
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu \
  --no-pdf-header-footer --print-to-pdf="$PWD/report.pdf" "file://$PWD/report.html" 2>/dev/null
echo "report.pdf written"
