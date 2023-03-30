data = readtable('enablev2_1.csv');
data1 = readtable('enablev2_2.csv');
data2 = readtable('enablev2_3.csv');
data3 = readtable('enablev2_4.csv');
data4 = readtable('enablev2_5.csv');
data5 = readtable('enablev2_6.csv');
data6 = readtable('enablev2_7.csv');


%1
figure(1)
plot(data.Var4/1000,data.Var1,data.Var4/1000,data.Var2,data.Var4/1000,data.Var3*5)

%5
figure(2)
plot(data1.Var4/1000,data1.Var1,data1.Var4/1000,data1.Var2,data1.Var4/1000,data1.Var3*5)

%50
figure(3)
plot(data2.Var4/1000,data2.Var1,data2.Var4/1000,data2.Var2,data2.Var4/1000,data2.Var3*5)

%dubble 5
figure(4)
plot(data3.Var4/1000,data3.Var1,data3.Var4/1000,data3.Var2,data3.Var4/1000,data3.Var3*5)

%dubble 10
figure(5)
plot(data4.Var4/1000,data4.Var1,data4.Var4/1000,data4.Var2,data4.Var4/1000,data4.Var3*5)

%dubble 50
figure(6)
plot(data5.Var4/1000,data5.Var1,data5.Var4/1000,data5.Var2,data5.Var4/1000,data5.Var3*5)

%dubble 15
figure(7)
plot(data6.Var4/1000,data6.Var1,data6.Var4/1000,data6.Var2,data6.Var4/1000,data6.Var3*5)
