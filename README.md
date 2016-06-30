# PSMove_Websocket
Connect to PSMove controller and send its data to websocket server

This project is run on the Raspberry Pi 3 and its main functionality is, using psmoveapi, Websocketpp and rapidjson, to poll data from
PS Move controllers that are connected to the Raspberry Pi 3 over bluetooth and send the data, in JSON format, to a websocket server.

Still a work in progress, adding the functionality to send commands from the server to control the Psmove LED and rumble motor among other thigs.
