# Pathfinding

一个全面的 Go 寻路库，支持多种网格类型和搜索算法。

## 特性

### 寻路算法

| 算法 | 双向搜索 | 说明 |
|-----------|:-------------:|-------------|
| **A\*** | ✅ | 经典启发式最短路径 |
| **Dijkstra** | ✅ | 加权最短路径（无启发式） |
| **Best-First Search** | ✅ | 贪心最佳优先搜索 |
| **Breadth-First Search** | ✅ | 广度优先遍历 |
| **IDA\*** | ❌ | 迭代加深 A\*，内存占用极低 |
| **JPS (Jump Point Search)** | ❌ | 跳点搜索，利用网格对称性加速 |

### 网格类型

- **正交网格 (Orthogonal)** — 标准方格网格
- **六边形网格 (Hexagonal)** — 六边形瓦片网格（轴向坐标）
- **交错网格 (Staggered)** — 45 度等距/交错瓦片网格

### 高级系统

- **HPA\* (Hierarchical Pathfinding A\*)** — 层级寻路，通过抽象分层加速大地图寻路
- **Waypoint Graph** — 航点图寻路，基于预计算节点连通性

## 安装

```bash
go get github.com/actfuns/pathfinding
```

## 快速开始

```go
package main

import (
    "fmt"
    "github.com/actfuns/pathfinding/grid"
)

func main() {
    // 定义地形：0 = 可通行，非零 = 障碍物
    terrain := [][]int{
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0},
    }

    // 创建正交网格（默认瓦片大小 1×1 世界单位）
    g := grid.NewOrthogonalGrid(terrain)

    // 在世界坐标系中寻路
    path := g.FindPath(0, 0, 4, 4)
    fmt.Println("路径:", path)

    // 或使用路径平滑
    // smooth := g.FindSmoothPath(0, 0, 4, 4)

    // 在运行时修改地形
    g.SetWalkableAt(2, 2, false)
}
```

## 文档

- [Finder API](finder/finder.go) — 所有寻路算法的通用接口
- [Grid](finder/grid.go) — 网格表示和地形定义
- [网格实现](grid/) — 正交、六边形、交错三种网格类型
- [HPA\*](hpa/) — 层级寻路
- [Waypoint](waypoint/) — 航点图寻路

## 许可

MIT License — 详见 [LICENSE](LICENSE) 文件。