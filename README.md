updated for new changes in kernel > 3.2. it's just a dirty job to make things running again. v4l2 supports is not fully implemented. so just use it like before (in old days, with our old machines :).

if your kernel version <= 3.2, you may go here: https://code.google.com/p/r5u870/ , and find what you want under "Downloads" tab.

how to install:

1. download as zip (the easiest way)

2. extract it into a folder

3. cd into the folder, cd into "r5u870", and type "make"

4. type "sudo make install"

5. type "sudo modprobe r5u870" (or any other methods to load driver)

6. try your webcam

for skype users: plz downgrade your skype to 2.1, you could download here:

http://download.skype.com/linux/skype-debian_2.1.0.81-1_i386.deb http://download.skype.com/linux/skype-debian_2.1.0.47-1_i386.deb

or

http://download.skype.com/linux/skype_static-2.1.0.81.tar.bz2
