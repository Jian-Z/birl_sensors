#FT_WACOH

1. Connect the FT Sensor
- Make sure to power your Wacoh FT sensor, and plugin your ethernet cable to your workstation.
- The Wacoh FT Sensor requires an ethernet connection. Go to your ubunut internet connection icon, and select the ethernet connection option.

2. Setup your IP Configuration
Under your IP settings, under Ipv4, use the following settings:
````
10.0.0.2
255.255.255.0
0.0.0.0
````

3. Check your connection
- Open a terminal:
- Ping the FT sensor by issuing the following command:
````
ping 10.0.0.5
````

4. Make the Node
- After you have verified your connection, go ahead and catkin_make your node.

5. Run the Node
- The node requires: (i) an ip number, (ii) a port number, and (iii) an offset command. The node can be run as follows: 
````
rosrun FT_WACOH minimal_publisher2 10.0.0.5 5001 OOO
````
