Beltway1: 2301 reference poses over 230.0 s, path 1764 m
Beltway2: 2207 reference poses over 220.6 s, path 1743 m
Beltway3: 2112 reference poses over 211.1 s, path 1745 m
Marketplace3: 1911 reference poses over 191.0 s, path 944 m
Tunnel2: 783 reference poses over 78.2 s, path 392 m
UndergroundCarPark1: 1783 reference poses over 178.2 s, path 308 m
UndergroundCarPark2: 1431 reference poses over 143.0 s, path 281 m

Preregistered score (median of the completed repetitions, m):

| sequence | G (G22) | RKO-LIO upstream | BIEVR-LIO | FAST-LIO2 | GLIM | G / best rival |
|---|---|---|---|---|---|---|
| Beltway1 | 1.6652 | 1.3074 | 1.4697 | 3.1513 | 2.6914 | 1.27 |
| Beltway2 | 1.4886 | 1.1794 | 1.4399 | 2.9561 | 2.5056 | 1.26 |
| Beltway3 | 1.5606 | 1.2735 | 1.4947 | 2.8957 | 2.4876 | 1.23 |
| Marketplace3 | 0.4982 | 0.4058 | 0.9890 | 0.5739 | 0.3992 | 1.25 |
| Tunnel2 | 1.5400 | 1.2599 | 0.2924 | 0.4147 | 1.4052 | 5.27 |
| UndergroundCarPark1 | 0.1061 | 0.0952 | 0.0863 | 0.0897 | 0.0857 | 1.24 |
| UndergroundCarPark2 | 0.1314 | 0.1122 | 0.1047 | 0.1144 | 0.1131 | 1.26 |

H9 (7 sequences): geometric means over each method's completed sequences: G (G22) 0.6345, RKO-LIO upstream 0.5231, BIEVR-LIO 0.5041, FAST-LIO2 0.6781, GLIM 0.7102
Over the 7 sequences every method completed: G (G22) 0.6345, RKO-LIO upstream 0.5231, BIEVR-LIO 0.5041, FAST-LIO2 0.6781, GLIM 0.7102
G more than 10 % worse than its best rival (or failed) on 7 of 7: Beltway1, Beltway2, Beltway3, Marketplace3, Tunnel2, UndergroundCarPark1, UndergroundCarPark2

| method | completed runs | worst RTF | worst peak RSS (MB) |
|---|---|---|---|
| G (G22) | 21/21 | 0.99 | 463 |
| RKO-LIO upstream | 21/21 | 1.04 | 178 |
| BIEVR-LIO | 21/21 | 1.38 | 4044 |
| FAST-LIO2 | 21/21 | 1.15 | 1032 |
| GLIM | 21/21 | 0.43 | 1904 |

G per sequence (worst of three): Beltway1 RTF 0.78 / 408 MB, Beltway2 RTF 0.97 / 431 MB, Beltway3 RTF 0.99 / 430 MB, Marketplace3 RTF 0.90 / 463 MB, Tunnel2 RTF 0.97 / 445 MB, UndergroundCarPark1 RTF 0.65 / 442 MB, UndergroundCarPark2 RTF 0.55 / 424 MB

Diagnostic, not the preregistered score: ATE with one constant time offset per run (every run with a finite trajectory, complete or not), and the median offset chosen (s):

| sequence | G (G22) | RKO-LIO upstream | BIEVR-LIO | FAST-LIO2 | GLIM | G / best rival |
|---|---|---|---|---|---|---|
| Beltway1 | 1.6550 | 1.3074 | 1.4604 | 3.1489 | 2.6887 | 1.27 |
| Beltway2 | 1.4731 | 1.1771 | 1.4384 | 2.9475 | 2.5056 | 1.25 |
| Beltway3 | 1.5527 | 1.2735 | 1.4858 | 2.8956 | 2.4832 | 1.22 |
| Marketplace3 | 0.4981 | 0.4055 | 0.9825 | 0.5734 | 0.3992 | 1.25 |
| Tunnel2 | 1.3946 | 1.1800 | 0.2798 | 0.3850 | 1.1548 | 4.98 |
| UndergroundCarPark1 | 0.1045 | 0.0937 | 0.0858 | 0.0897 | 0.0851 | 1.23 |
| UndergroundCarPark2 | 0.1271 | 0.1070 | 0.0957 | 0.1042 | 0.1049 | 1.33 |

| sequence | G (G22) | RKO-LIO upstream | BIEVR-LIO | FAST-LIO2 | GLIM |
|---|---|---|---|---|---|
| Beltway1 | +0.020 | +0.000 | -0.020 | -0.005 | -0.015 |
| Beltway2 | +0.025 | +0.010 | -0.010 | +0.015 | +0.000 |
| Beltway3 | +0.015 | +0.000 | -0.020 | -0.005 | -0.015 |
| Marketplace3 | -0.005 | -0.005 | +0.025 | +0.010 | +0.000 |
| Tunnel2 | +0.140 | +0.105 | -0.020 | +0.025 | +0.150 |
| UndergroundCarPark1 | -0.010 | -0.010 | -0.005 | -0.005 | -0.005 |
| UndergroundCarPark2 | +0.020 | +0.020 | +0.025 | +0.030 | +0.025 |

Spans (median fraction of the reference time span): Beltway1 G 1.000 rko_upstream 1.000 bievr 0.999 fastlio 0.998 glim 0.987; Beltway2 G 1.000 rko_upstream 1.000 bievr 0.999 fastlio 0.998 glim 0.986; Beltway3 G 1.000 rko_upstream 1.000 bievr 0.999 fastlio 0.998 glim 0.986; Marketplace3 G 1.000 rko_upstream 1.000 bievr 0.999 fastlio 0.998 glim 0.984; Tunnel2 G 0.999 rko_upstream 0.999 bievr 0.997 fastlio 0.995 glim 0.962; UndergroundCarPark1 G 0.999 rko_upstream 0.999 bievr 0.999 fastlio 0.998 glim 0.983; UndergroundCarPark2 G 0.999 rko_upstream 0.999 bievr 0.999 fastlio 0.997 glim 0.978

H8 + H9: geometric means over each method's completed sequences: G (G22) 0.1430 (23 sequences), RKO-LIO upstream 0.1477 (23 sequences), BIEVR-LIO 0.1506 (23 sequences), FAST-LIO2 0.1957 (23 sequences), GLIM 0.2288 (20 sequences)
Over the 20 sequences every method completed: G (G22) 0.1687, RKO-LIO upstream 0.1704, BIEVR-LIO 0.1701, FAST-LIO2 0.2296, GLIM 0.2288
