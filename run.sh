#!/usr/bin/env bash

echo Starting BWO!

cd "$(dirname "$0")"

source bin/activate

export PYTHONPATH=/usr/lib/python3.6/dist-packages/

cd src/python

exec python -u main.py
