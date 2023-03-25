
if [ $# -ne 2 ];then
    echo "引数を２つ指定してください"
    exit  0
fi

# 横壁から5枚で35 ~ 65
# 前壁から7枚で45 ~ 147

filename="${1}_${2}.csv"
# node tools/param_tuner/sensor.js $1 $2
node tools/param_tuner/sensor.js $1 $2>$filename