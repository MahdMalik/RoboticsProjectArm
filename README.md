TO RUN:

Get into the jetson somehow. If you've set up the wifi connection and know its ip addr, you can sssh as normla.

Otherwise, you can power the jetson and plug it into a hdmi monitor, and attach a usb with a keyboard + mouse so you can actually go into the jetson's desktop. From there you can add a wifi and get its ip addr so you can SSH into it later.

Next once you're in the jetson, in Documents/CS6341Proj you can find the repo. Make sure you do git pull / push if you have any changes / need to receive any changes, and also make sure you activate the virtual environment with "source .venv/bin/activate". 

Now for actually running the scripts, you can find all the current dev scripts in src/robotics. Just run them as nomral with "python fileName.py".