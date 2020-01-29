#!/bin/bash
raspivid -t 0 -fps 30 -o - | nc -u -l -k 1234
