##Sending data from PS Move controller to NI Mate using PSMoveApi & websockets

###Project goals
This project was created to show the possibilities of using websockets in conjunction with [NI Mate](https://ni-mate.com/), to use data from other devices, in this case a PS Move controller, for additional tracking. Another project which was made at the same time has the function of controlling a vibrating motor through the NI Mate software. This documentation will focus on how the project is built and how the program is used. The code is not in any way elegant, mostly due to my limited knowledge in the C++ language. Feel free to change the code to your own liking. Even though the code was made to work with [NI Mate](https://ni-mate.com/), it can be used with any websocket server as long as the server can send a "start" message back to the client so that the program knows when to start sending data. The software should work on OS X, Linux and Windows but I have only tested it on Linux so far.

#####Devices used in my test
- Raspberry Pi 3
- Playstation Move Motion Controller
- Playstation Eye camera (Optional)

#### Software / Libraries used
- Raspbian GNU / Linux 8 (Jessie)
- [PSMoveApi](https://thp.io/2010/psmove/)
- [Boost](http://www.boost.org/)
- [Websocket++](https://github.com/zaphoyd/websocketpp)
- [Nlohmann Json](https://github.com/nlohmann/json)
- g++ 4.9 or higher

####Initial configuration (Linux / Raspberry Pi)
We start by updating the OS with the following commands:

- `sudo apt-get update`
- `sudo apt-get upgrade`
- `sudo reboot`

We update the Raspberry Pi firmware:

- `sudo apt-get install rpi-update`
- `sudo rpi-update`
- `sudo reboot`

To get the Bluetooth to work, we run the following commands:	(Only applies to the Raspberry Pi)

- `sudo apt-get install pi-bluetooth`
- `sudo apt-get install bluez bluez-firmware`
- `sudo apt-get install blueman`
- `sudo usermod -G bluetooth -a pi`
- `sudo reboot`

After that we will install our libraries by following the installation instructions online:

- [PSMoveApi](https://media.readthedocs.org/pdf/psmoveapi/latest/psmoveapi.pdf)
- [Websocket++](https://github.com/zaphoyd/websocketpp/wiki/Build-on-debian)
- Boost: `sudo apt-get install libboost-all-dev` (Also good to have)
- Download the websocket++ repo from github and extract it to `/home/pi/`
- [Nlohmann Json](https://github.com/nlohmann/json), download project, place the json.hpp file into our project folder:
- `cd ~/Downloads`
- `git clone https://github.com/nlohmann/json`

Link the libraries, add the following lines to `/etc/ld.so.conf.d/libc.conf`:

- `sudo nano /etc/ld.so.conf.d/libc.conf`
- `/usr/local/boost_1_61_0/libs` (If newer verions was installed, change name accordingly)
- `/usr/local/boost_1_61_0/libbin` (If newer verions was installed, change name accordingly)
- `/home/pi/src/psmoveapi/build`

Add the following line to `/etc/ld.so.conf.d/local.conf`

- `sudo nano /etc/ld.so.conf.d/local.conf`
- `/usr/local/boost_1_61_0/libbin/lib` (If newer verions was installed, change name accordingly)
- run `sudo ldconfig`

After all the libraries that we need have been installed, we can begin to pair our controllers with our Raspberry Pi / Computer. To do this we will run the built in program that comes with the PSMoveApi.

- Connect the controller to the USB port on the Raspberry Pi
- `cd src/psmoveapi/build/`
- `sudo ./psmovepair`

Once all the controllers have been paired we can start to calibrate the controllers. Start by connecting the controller to the computer via Bluetooth (if not already connected), this can take a couple of tries. Once a controller is connected we can start to calibrate it by running the following command:

- while in the same directory as the psmovepair:
- `sudo ./magnetometer_calibration`
- Follow the on-screen instructions

After all the controllers have been paired and calibrated we can start to download this project. All code can be found at the following link: [GitHub: PSMove Websocket](https://github.com/Pete-22/PSMove_Websocket/).

- `cd ~/Downloads`
- We can use `git clone https://github.com/Pete-22/PSMove_Websocket/` to download the project
- Next we need to copy the following files from the psmoveapi folder to our project folder:
- `cp ~/src/psmoveapi/examples/c/psmove_examples_opengl.h ~/Downloads/PSMove_Websocket`
- `cp ~/src/psmoveapi/include/psmove.h ~/Downloads/PSMove_Websocket`
- `cp ~/src/psmoveapi/include/psmove_config.h ~/Downloads/PSMove_Websocket`
- `cp ~/src/psmoveapi/include/psmove_fusion.h ~/Downloads/PSMove_Websocket`
- `cp ~/src/psmoveapi/include/psmove_tracker.h ~/Downloads/PSMove_Websocket`

Next we can start to compile the project code:

- `cd ~/Downloads/PSMove_Websocket`
- Use the makefile to compile the code (The makefile might need to be edited according to the username of your system!).
- `make`
- The compiling can take a couple of minutes on a Raspberry Pi, seconds on a desktop computer.
- Once the program has been compiled we can run it with the command:
- `sudo ./psmove_send_poll_data`

####Using the program
- The program will first print out how many controllers are connected to the computer and after that, ask if you want to use the PS Eye camera to track the controller(s).
- If you want to use tracking press _"**y**"_ and then _"**enter**"_. If you don't have a camera or don't want to use tracking, just press _"**n**"_ and then _"**enter**"_.
- Once tracking has been either chosen or not, the program will ask for NI Mates (or the websocket servers) IP address and port.
- Input the IP address and port according to the example on screen and then press _"**enter**"_.
- Next the program will connect to the server, send an initial message and then wait for the start message to be received from the server.
- After receiving the start message the program will start to poll the controllers and send the data to the server.
- NI Mate will now have a new tab where the data can be found, as well as the different commands that can be sent to the controller(s).
- To pause the program, you can send a _"**pause**"_ command from NI Mate
- To stop the program, either press the PS button on the controller or then you can _"**stop**"_ the sensor from NI Mate

