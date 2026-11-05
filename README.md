TO RUN:

If controlling from a computer:
- do git clone as normal
- make a virtual environment in the repo, do "pip install torch torchvision" and "pip install -e ".[core_scripts,feetech]"
- when trying to find the correct port, first run "lerobot-find-port", follow the instructions ans it'll tell you what com port it's on. No calibration stuff needs to be done
- finally to test, just run the python file in src/robotics/motor-test.py


If using a Jetson Nano:
Get into the jetson somehow. If you've set up the wifi connection and know its ip addr, you can sssh as normla.

Otherwise, you can power the jetson and plug it into a hdmi monitor, and attach a usb with a keyboard + mouse so you can actually go into the jetson's desktop. From there you can add a wifi and get its ip addr so you can SSH into it later.

Next once you're in the jetson, in Documents/CS6341Proj you can find the repo. Make sure you do git pull / push if you have any changes / need to receive any changes, and also make sure you activate the virtual environment with "source .venv/bin/activate". 

Now for actually running the scripts, you can find all the current dev scripts in src/robotics. Just run them as nomral with "python fileName.py".
