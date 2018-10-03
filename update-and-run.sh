#!/bin/sh

echo Pulling changes...
git pull

echo Installing dependencies...
pip3 install -r requirements.txt

echo Starting BWO!
python3 bwo.py