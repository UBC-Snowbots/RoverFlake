
	Linux is a very powerful enviroment. Installing can be easy, but depending on your computer, and your luck, a variety of errors can occur.
	
	There is a chance you might mess up the software and/or firmware of your computer. This means that installing linux may slightly break your windows or mac envirowment, so make sure to backup any important files. These errors should be minor, but a hard reset should fix your computer if all else fails.
	
The most stable way to install linux is to remove other operating systems and have linux as your primary and only OS.
	-This means you won't be able to use Windows or Mac, and you will lose all of your data when installing

Another stable way is to dual-boot. This is what most people choose, and is pretty much as stable as only installing linux
	-You will need to virtually split your hard drive. I reccomend a minimum of 100gb to comfortably develop with ROS (including visualizers, simulators, and extra packages), but you can get away with less if needed.
		- This is called partitioning your drive: https://support.microsoft.com/en-us/windows/create-and-format-a-hard-disk-partition-bbb8e185-1bda-ecd1-3465-c9728f7d7d2e
	-Once your hard drive split, you will have free space. If you have a boot usb, you can restart your computer now and boot into the ubuntu image
	-We have bootable usbs with the right linux version in the shop, but you can make one yourself too: https://etcher.balena.io/#download-etcher
		- tutorial: https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
	-make sure to install ubuntu 20.04, "Focal Fossa". 20.04 is what works with ROS Noetic

You can also use a Virtual Machine, and this is just installing an app on your primary OS that acts as a full ubuntu enviroment
 	-tutorial: https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview


