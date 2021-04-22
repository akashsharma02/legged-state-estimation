Might need to build cxxopts, included in thirdparty,

```
mkdir build
cd build
cmake ..
make
```

Then 
```
./main -m 15000 -f ../data/sequence1.txt
OR
./main -m 24000 -f ../data/sequence2.txt
```

And then from ./python

```
python raw_data_vis.py -p ../build/result.csv
```