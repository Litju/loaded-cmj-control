#!/bin/bash
# RES-52 fresh-process reproduction of the soft-contact qualification.
set -e
cd /home/litju/Projects/loaded-cmj-control
PY=.venv/bin/python
$PY tools/res52/reproduce_res52.py
