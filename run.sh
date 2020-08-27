#!/usr/bin/env bash

echo Starting BWO!

cd "$(dirname "$0")"

source bin/activate

cd src/python

exec python -u main.py
