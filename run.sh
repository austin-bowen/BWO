#!/usr/bin/env bash

echo Starting BWO!

cd "$(dirname "$0")"

. bin/activate

cd src/python

python main.py
