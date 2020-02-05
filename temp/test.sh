#!/bin/bash
# RPR-0521RS Reading distance
while [[ true ]];
do
    sleep 1;
    # set control registers according to datasheet and user settings
    # i2cset -y 1 0x38 0x42 0x02
    # i2cset -y 1 0x38 0x43 0x00
    # i2cset -y 1 0x38 0x41 0xC6

    # ambient light measurement
    valueA0="0x01"
    valueA1="0x04"
    resultA=$(( $(($valueA0)) * 256 + $(($valueA1)) ));
    echo "resultA =" $resultA;

    valueB0="0x01"
    valueB1="0x15"
    resultB=$(( $(($valueB0)) * 256 + $(($valueB1)) ));
    echo "resultB =" $resultB;

    if [[ $((resultA)) == 0 ]];
    then
        echo "resultA =" 0;
    else
        getVal=0;
        data=$(( $(($resultB)) * 1000 / $(($resultA)) ));
        echo "data =" $data;

        if [ $data -lt 595 ]
        then
            echo "OPT 1";
            getVal=$(( resultA * 1682 - resultB * 1877 ));
        elif [ $data -lt 1015 ]
        then
            echo "OPT 2";
            getVal=$(( resultA * 644 - resultB * 132 ));
        elif [ $data -lt 1352 ]
        then
            echo "OPT 3";
            getVal=$(( resultA * 756 - resultB * 243 ));
        elif [ $data -lt 3053 ]
        then
            echo "OPT 4";
            getVal=$(( resultA * 766 - resultB * 250 ));
        fi
        echo $getVal;
        echo $((getVal/1000)).$((getVal%1000));
    fi
    

    # proximity measurement
    valueC0="0x06"
    valueC1="0x08"
    resultC=$(( $(($valueC0)) * 256 + $(($valueC1)) ));
    echo "resultC =" $resultC;
    echo;
done