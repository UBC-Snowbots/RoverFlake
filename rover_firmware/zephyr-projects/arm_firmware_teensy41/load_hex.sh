echo LOADING HEX
teensy_loader_cli -w -v --mcu TEENSY41 build/zephyr/zephyr.hex 
sleep 1.5
teensy_loader_cli -w -v --mcu TEENSY41 build/zephyr/zephyr.hex 

