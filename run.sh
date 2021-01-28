#!/usr/bin/env bash

echo Starting BWO!

cd "$(dirname "$0")"

source bin/activate

cd src/python

SERIAL_PACKETS_LIB=~/Projects/serial-packets/src/python/
export PYTHONPATH=$SERIAL_PACKETS_LIB:/usr/lib/python3.6/dist-packages/

exec python -u main.py
