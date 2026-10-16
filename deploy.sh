#!/bin/sh
# Rebuild the viewer pages from web/ and deploy budotsmedia.com/uas/dji-dock3-matrice4d/ (goes live at once).
set -e
cd "$(dirname "$0")"
python3 web/build_viewer.py
npx wrangler deploy
