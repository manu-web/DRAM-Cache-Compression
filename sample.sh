#!/bin/sh
make simulate BENCH=leslie3d BBV=1
make simpoint BENCH=leslie3d
make simulate BENCH=leslie3d CHKP=1