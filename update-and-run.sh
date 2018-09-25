#!/bin/sh

echo Pulling changes...
git pull

echo Starting BWO!
python3 bwo.py