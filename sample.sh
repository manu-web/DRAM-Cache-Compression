#!/bin/sh
make simulate FF=1 DC=1 BENCH=gcc &
make simulate FF=1 DC=1 BENCH=leslie3d &
make simulate FF=1 DC=1 BENCH=soplex &