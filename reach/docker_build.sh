#!/bin/sh
PWD=`pwd`
cd `dirname $0`
BASEDIR=`pwd`
docker run --mount type=bind,src=$BASEDIR,target=$BASEDIR pladevbase $BASEDIR/build.sh
cd $PWD
