follow the vrx/aquabot project installation requirement:

https://github.com/sirehna/Aquabot#installation

https://github.com/sirehna/Aquabot-Competitor#installation

for the project dependency we managed to only add a single library, for the detection of qr code, Zbar:

→    “sudo apt-get update”
→    “sudo apt-get install libzbar-dev”

to run our code, don’t forget to source your installation “bashrc/zshrc/...” and launch your sim world.

→    “ros2 run all_star BabySharky”

warning: our code needs to be re-launched whenever a windTurbin has been scanned, or it won’t find them on the next simulation run !
