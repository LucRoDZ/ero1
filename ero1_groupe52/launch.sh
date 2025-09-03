#!/bin/bash

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python3 src/display.py

echo -e "3\n2"
