#!/bin/sh

echo Pulling changes...
git pull
wget https://raw.githubusercontent.com/SaltyHash/PycoTTS/master/pycotts.py

echo Installing dependencies...
pipenv install -r requirements.txt
