#! /bin/bash

COUNTER=0
while [ $COUNTER -lt 4000 ]
do
    echo $COUNTER
    let COUNTER++
done
