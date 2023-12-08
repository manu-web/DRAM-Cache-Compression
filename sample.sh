#!/bin/sh
make simulate BENCH=gcc BBV=1 && make simpoint BENCH=gcc && make simulate BENCH=gcc CHKP=1