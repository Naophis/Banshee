
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