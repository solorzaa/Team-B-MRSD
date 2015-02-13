#!/bin/bash

cat matlabLog | grep absoluteX | tee aX
cat matlabLog | grep absoluteY | tee aY
cat matlabLog | grep absoluteTh | tee aT
cat matlabLog | grep current | tee cT
cat matlabLog | grep linear | tee lv
cat matlabLog | grep angular | tee av
