# Micromouse Software Setup 2025

# Table of Contents

1. [Setup Git Repo](#setup-git-repo)

2. [Install Arduino IDE](#install-arduino-ide)
    1. [Enable Teensy Support](#enable-teensy-support)
3. [Install Mouse Simulator](#install-mouse-simulator)
    1. [Install C Compiler GCC](#install-c-compiler-gcc)
    2. [Run the Simulator](#to-run-the-simulator)
    3. [OPTIONAL - Install QT Creator](#optional-install-qt-creator)

# Setup Git Repo
If you already have Git or Git Desktop installed, you can skip to [Cloning the Git Repo](#cloning-the-git-repo)  
If you want do version control from the command line and forgo opening another application to push your changes to the repository, just download [Git](https://git-scm.com/downloads).  
Otherwise, if you'd like to forgo using the command line and use a GUI instead, install [GitHub Desktop](https://desktop.github.com/download/) in addition to Git.  
## Cloning the Git Repo
Open up a terminal like `Git Bash`, and `cd` from the current directory to the directory that you want to have the repository
```bash
cd folder/folder/desired_folder
```
And from there, type this into your terminal to clone the repo
```bash
git clone https://github.com/CWRUbotix/Micromouse-2023-24.git
```
You should now have a folder called `Micromouse-2023-24` in your current directory
# Install Arduino IDE
If you don't already have it installed, go [here](https://www.arduino.cc/en/software) to install the Arduino IDE
## Enable Teensy Support
Steps to enable Teensy Support are [here](https://www.pjrc.com/arduino-ide-2-0-0-teensy-support/).  
After following the instructions on that page, click `File` and click `Preferences`.  
Then click the `Browse` button next to `Sketchbook Location`.  
This will determine where your Arduino libraries and sketches are saved.  
You should go ahead and change this to where you have the Micromouse repository saved.  
# Install Mouse Simulator
To install the simulator we currrently use for the mouse, go [here](https://github.com/mackorone/mms#download), note the instructions forr each operating system.  
Then click the link to `releases` and install it for windows/mac.  
Once you have it installed, you'll need to extract it. If you don't have WinRAR installed, you may be able to use windows file explorer to extract it. Otherwise, you'll need to install [WinRAR](https://www.win-rar.com/download.html?&L=0)  
## Install C Compiler GCC
To run the simulator, you need to install the C compiler GCC.  
To do so, follow the instructions on this cool [medium article](https://dev.to/gamegods3/how-to-install-gcc-in-windows-10-the-easier-way-422j).  
### To run the Simulator
Navigate to the mms folder you installed and double click on `mms.exe`  
From there, click the plus icon under `Config` in the top right, and fill in the boxes according to the image below  
![Img](https://cdn.discordapp.com/attachments/432337516742443018/1284202534000529469/image.png?ex=66e5c64f&is=66e474cf&hm=75f797a8cb5c9f6259f6262ab7749bad30986cb56aabcd0f02875b4f16c258d1&)

If you click `OK`, then click `Build` and `Run`, it should run the currrent algorithm we have for the robot.