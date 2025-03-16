# git submodule update --init --recursive
BOARD=MatekH7A3
echo "building for $BOARD"
#./waf configure --board $BOARD
#./waf distclean
./waf copter
cp ./build/$BOARD/bin/arducopter.apj ~/Desktop/
cp ./build/$BOARD/bin/arducopter_with_bl.hex ~/Desktop/
