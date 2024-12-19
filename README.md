#BHGA_CEVRP:A Bilevel Hybrid Genetic Algorithm for Capacitated Electric Vehicle Routing Problem
##How to use

Build

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles"
make bin
```
Run

```bash
E-n22-k4 mySolution.sol -it 30000 -seed 35 -mix 12116
```

Options are as follow:

```bash
[-it]
iterations without improvement
[-mix]
12100:BHGA-SO in paper[1], with step=0
12110:BHGA-ST in paper[1], with step=10
12116:BHGA-SP in paper[1], with step=6
```

##References
If you like this project, please cite the following works:
[1] Feng, C. T., Jia, Y. H., Yang, Q., Chen, W. N., & Jiang, H. (2024, June). A Bilevel Hybrid Genetic Algorithm for Capacitated Electric Vehicle Routing Problem. In 2024 IEEE Congress on Evolutionary Computation (CEC) (pp. 1-8). IEEE.
[2] Vidal, T. (2022). Hybrid genetic search for the CVRP: Open-source implementation and SWAP* neighborhood. Computers & Operations Research, 140, 105643.
[3] Jia, Y. H., Mei, Y., & Zhang, M. (2021). A bilevel ant colony optimization algorithm for capacitated electric vehicle routing problem. IEEE Transactions on Cybernetics, 52(10), 10855-10868.
[4] Jia, Y. H., Mei, Y., & Zhang, M. (2022). Confidence-based ant colony optimization for capacitated electric vehicle routing problem with comparison of different encoding schemes. IEEE Transactions on Evolutionary Computation, 26(6), 1394-1408.
[5] Vidal, T., Crainic, T. G., Gendreau, M., Lahrichi, N., & Rei, W. (2012). A hybrid genetic algorithm for multidepot and periodic vehicle routing problems. Operations Research, 60(3), 611-624.
