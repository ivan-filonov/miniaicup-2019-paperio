#!/bin/bash
set -ex
gdb cpp1 -ex "r < ../../../local_runner/inputs.txt"
