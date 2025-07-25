# kalmanFilterForLaneParam
kalmanFilterForLaneParam
# RUN
cd build
cmake ..
make
./kalman

```
$ ./kalman 
matrix_P_ = 0.001     0     0     0
    0 0.001     0     0
    0     0 0.001     0
    0     0     0 0.001
matrix_X_init =   1.8
  0.1
0.001
1e-06
meaasurement matrix_Z =  1.95
 0.13
0.006
1e-06
predict x_:  1.98162
 0.101802
0.0010018
    1e-06
matrix_X_res =      1.9808
   0.102254
 0.00150389
0.000227482
meaasurement matrix_Z =  2.25
 0.14
0.007
1e-06
predict x_:    2.16752
    0.10533
 0.00191336
0.000227482
matrix_X_res =    2.21254
  0.124447
 0.0299768
0.00420442
meaasurement matrix_Z =  2.55
 0.15
0.008
1e-06
predict x_:    2.4892
  0.185216
 0.0375448
0.00420442
matrix_X_res =    2.59024
  0.114731
0.00890747
 0.0188586
meaasurement matrix_Z =  2.85
 0.16
0.009
1e-06
predict x_:  2.82951
 0.161315
 0.042853
0.0188586
matrix_X_res =    2.90133
 0.0843258
 -0.019512
-0.0318209
meaasurement matrix_Z =  3.15
 0.17
 0.01
1e-06
predict x_:    2.99058
-0.00234559
 -0.0767896
 -0.0318209
matrix_X_res =      3.15827
    0.153481
   0.0105911
-5.43506e-05
meaasurement matrix_Z =  3.45
 0.18
0.011
1e-06
predict x_:     3.45165
    0.172457
   0.0104932
-5.43506e-05
matrix_X_res =     3.44968
   0.179362
   0.011441
0.000531953
meaasurement matrix_Z =  3.75
 0.19
0.012
1e-06
predict x_:    3.79158
   0.200817
  0.0123986
0.000531953
matrix_X_res =    3.75863
  0.190428
0.00116688
 0.0045454
meaasurement matrix_Z =  4.05
  0.2
0.013
1e-06
predict x_:   4.10771
  0.199892
0.00934859
 0.0045454
matrix_X_res =     4.08832
   0.119601
  0.0488686
-0.00152421
meaasurement matrix_Z =  4.35
 0.21
0.014
1e-06
predict x_:    4.38129
   0.205096
   0.046125
-0.00152421
matrix_X_res =     4.35049
   0.213435
  0.0114594
-0.00180987
meaasurement matrix_Z =  4.65
 0.22
0.015
1e-06
predict x_:    4.75148
    0.23113
 0.00820167
-0.00180987
matrix_X_res =    4.68102
  0.195486
0.00776116
-0.0024654
```

# More info
https://blog.csdn.net/qq_40464599/article/details/144333110?sharetype=blogdetail&sharerId=144333110&sharerefer=PC&sharesource=qq_40464599&spm=1011.2480.3001.8118
