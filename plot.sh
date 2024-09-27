#source tools/rosws/install/setup.bash
if [ $# -ne 0 ];then
    idx=$1
    if expr "$idx" : "[0-9]*$" >&/dev/null; then
        offset_idx=$(($idx+2))

        i=0
        for file in `ls -rt ./tools/param_tuner/logs/ | tail -n ${offset_idx}`
            do
            echo $file $i ${offset_idx}
            if [ $i -eq 0 ]; then
                echo ./tools/param_tuner/logs/$file
                read -p "Press [Enter] key to resume."
                `ros2 run plotjuggler plotjuggler -d ./tools/param_tuner/logs/$file -l ./tools/param_tuner/profile.xml`
                break
            fi
            i=$(($i+1))
        done
    else
        echo "not number"
    fi

else
    ros2 run plotjuggler plotjuggler -d ./tools/param_tuner/logs/latest.csv -l ./tools/param_tuner/profile.xml
fi
