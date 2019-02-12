% From the specification sheet
%
T=[0 
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
];
% Resistance in Ohms matchin above temperature
R=[32650.5
31032.1
29499.9
28052.4
26684.6
25391.2
24168.2
23011.2
21916.3
20879.8
19898.3
18968.6
18087.6
17252.6
16460.9
15710
14997.7
14321.6
13679.8
13070.4
12491.6
11941.6
11418.9
10922
10449.5
10000
9572.32
9165.29
8777.79
8408.68
8057.31
7722.43
7403.29
7098.42
6808.36
6531.31
6265.75
6016.47
5776.05
5546.53
5327.34
5117.97
4917.94
4726.77
4543.91
4369.33
4200.84
4040.81
3889.51
3743.17
3603.1
];

% Using 19.79 kOhm in series resistor we should get following counts on
% Arduino. The Analog In has 1024 alternatives.
A=1023*R./(R+19790);

% Now interpolate the Temperature as function of any possible analog
% reading of the Arduino.

Ai=[floor(min(A)):1:ceil(max(A))];
Ti=interp1(A,T,Ai','spline');

clf
plot(A,T,'o'); hold on
plot(Ai,Ti,'g'); 

Ti = round(Ti*100)/100;
sprintf('%0.2f, ',Ti)

min(Ai)
max(Ai)

% Make sure to use Ti in the lookup table of your Microcontroller code and
% Ai min/max as the min and max. Please take a look at how the lookup table
% works and update the values according your serial resistor.
