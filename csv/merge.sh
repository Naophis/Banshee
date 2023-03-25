rm -f result_l
rm -f result_r
rm -f result_f

echo "dist,L90,L45,F,R45,R90"> result_l.csv
echo "dist,L90,L45,F,R45,R90"> result_r.csv
echo "dist,L90,L45,F,R45,R90"> result_f.csv

cat *.csv | head -n 1 > result && find -name "l_*.csv" -exec sed -e '1d' {} \; >> result_l.csv
cat *.csv | head -n 1 > result && find -name "r_*.csv" -exec sed -e '1d' {} \; >> result_r.csv
cat *.csv | head -n 1 > result && find -name "f_*.csv" -exec sed -e '1d' {} \; >> result_f.csv