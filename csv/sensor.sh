
if [ $# -ne 2 ];then
    echo "引数を２つ指定してください"
    exit  0
fi

# 横壁から6枚で32(31)
 # 
# 前壁から8枚で42, 48, 84, 90, 96
 # 96から５枚除去
 # 126, 132, 138, 144(みえない)
 # 
filename="${1}_${2}.csv"
# node tools/param_tuner/sensor.js $1 $2
node ../tools/param_tuner/sensor.js $1 $2>$filename

# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 45
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 51
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 81
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 87
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 93
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 123
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 129
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 135
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ 
# naoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 141
# ^Cnaoto@naoto-omen-16:~/Desktop/mouse/Banshee/csv$ ./sensor.sh f 141