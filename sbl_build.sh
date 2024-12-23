# git submodule update --init --recursive
BOARD=FlywooF745
BOARD=CubeOrangePlus
BOARD=mRoControlZeroH7
BOARD=MatekH743
echo "building for $BOARD"
#./waf configure --board $BOARD
#./waf distclean
./waf copter
cp ./build/$BOARD/bin/arducopter.apj ~/Desktop/
cp ./build/$BOARD/bin/arducopter_with_bl.hex ~/Desktop/
