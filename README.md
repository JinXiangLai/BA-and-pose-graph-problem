# BA-program
仅使用Eigen库手写的一个SLAM BA求解程序，包括：

class KeyFrame & class Point：分别用于管理pose和point状态量；

class Map：用于管理KeyFrame和Point；

class NonLinearOptimizer：实现了LM算法；

class Simulator：用于产生数据以及打印评估结果。
