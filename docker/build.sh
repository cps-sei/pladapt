#!/bin/sh
PWD=`pwd`
cd `dirname $0`
docker build -t pladevbase .
cd $PWD
