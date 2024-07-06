# git submodule update --init --recursive
BOARD=FlywooF745
BOARD=CubeOrangePlus
echo "building for $BOARD"
# ./waf configure --board $BOARD
./waf copter
cp ./build/$BOARD/bin/arducopter.apj ~/Desktop/
