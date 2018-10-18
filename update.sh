#!/bin/sh



echo Pulling changes...
wget https://raw.githubusercontent.com/SaltyHash/PycoTTS/master/pycotts.py
git pull

echo Installing dependencies...
pip3 install -r requirements.txt

echo Starting BWO!
python3 bwo.py