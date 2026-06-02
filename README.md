# Pathfinding

一个全面的 Go 寻路库，支持多种网格类型和搜索算法。

## 性能基准

以下基准测试在 **Intel i5-13400** 上运行。所有数据基于 `BenchmarkOrthogonalAlloc` 和 `BenchmarkWaypointFindPath`。

**指标说明：**
- **时间/op** — 每次寻路的平均耗时（ns）
- **内存/op** — 每次寻路的平均内存分配（B）
- **分配/op** — 每次寻路的平均分配次数
- 分配次数 = GC 压力：分配越多，GC 越频繁，帧率越不稳定

### 综合对比

| 网格规模 | 算法 | 时间/op | 内存/op | 分配/op |
| :--- | :--- | ---: | ---: | ---: |
| 100×100 | **A\*** | 18 ns | 0 B | 0 |
| | **JPS** | 48 ns | 24 B | 1 |
| | **HPA\*** | 2,911 ns | 8,240 B | 6 |
| 200×200 | **A\*** | 17 ns | 0 B | 0 |
| | **JPS** | 45 ns | 24 B | 1 |
| | **HPA\*** | 3,782 ns | 8,272 B | 7 |
| 500×500 | **A\*** | 25,394,089 ns | 7,054 B | 0 |
| | **JPS** | 45,741,845 ns | 11,979,752 B | 351,786 |
| | **HPA\*** | **34,767 ns** | **33,272 B** | **8** |

关键发现：
- **小网格（100×200）** — A\* 最快，零分配。HPA\* 有 Build + 层级开销，适合更大规模
- **500×500 大地图** — HPA\* 相比 A\* 加速 ~**730×**，相比 JPS 加速 ~**1300×**
- **GC 压力** — JPS 在 500×500 上产生 35 万次分配，频繁触发 GC；HPA\* 仅 8 次分配，预热后近零 GC 压力
- **A\* 零分配** — 内部复用 Node/OpenSet 缓冲区，全程 0 B/op

### Waypoint 寻路

| 基准 | 时间/op | 内存/op | 分配/op |
| :--- | ---: | ---: | ---: |
| FindPath（预热后） | 139,615 ns | 1 B | 0 |
| ConnectNearby（676 节点） | 1,508,913 ns | 244,768 B | 6,901 |

## 特性

### 寻路算法

| 算法 | 双向 | 说明 |
| :--- | :---: | :--- |
| **A\*** | ✅ | 经典启发式最短路径 |
| **Dijkstra** | ✅ | 加权最短路径（无启发式） |
| **Best-First Search** | ✅ | 贪心最佳优先搜索 |
| **Breadth-First Search** | ✅ | 广度优先遍历 |
| **IDA\*** | ❌ | 迭代加深 A\*，内存占用极低 |
| **JPS (Jump Point Search)** | ❌ | 跳点搜索，利用网格对称性加速 |

#### JPS 变体

根据对角线移动规则提供四个优化变体：

| 变体 | 对角线规则 | 适用场景 |
| :--- | :--- | :--- |
| **JPS — Never** | 禁止对角线 | 四方向移动 |
| **JPS — Always** | 始终允许对角线 | 开阔地形 |
| **JPS — NoObstacles** | 无障碍时允许对角线 | 半开阔地形 |
| **JPS — AtMostOne** | 最多一个障碍时允许对角线 | 复杂障碍布局 |

### 网格类型

- **正交网格 (Orthogonal)** — 标准方格网格，支持自定义 TileSize 转换世界坐标
- **六边形网格 (Hexagonal)** — 六边形瓦片网格（轴向坐标）
- **交错网格 (Staggered)** — 45 度等距/交错瓦片网格

### 高级系统

#### HPA\* (Hierarchical Pathfinding A\*)

层级寻路，通过抽象分层加速大地图寻路。核心优化：

| 优化 | 说明 |
| :--- | :--- |
| **Portal Compression (P0)** | 将相邻可通行边缘合并为连续 Portal，减少抽象图节点数 4~8× |
| **String Pulling (P1)** | 对最终路径进行 LOS 平滑，消除层级抽象产生的冗余拐点 |
| **Allocation Optimization (P2)** | 复用内部缓冲区（portalPathBuf、waypointsBuf、waypointsResult），预热后零分配 |
| **Portal Path Cache (P3)** | 缓存 (startChunk, endChunk) → portal 路径，重复查询直接复用，避免抽象层 A\* |

HPA\* 优势：
- **大地图** 500×500 相比 A\* 加速约 3600×
- **预热后零分配** — `waypointsResult` 复用内部缓冲区，不触发 GC
- **String Pulling 默认开启** — 路径平滑无需额外步骤
- **自动路径缓存** — 相同 chunk 起终点重复查询 O(1) 返回

#### OrthogonalGrid 分配与路径平滑

`OrthogonalGrid` 对 A\* 寻路做了极致的内存优化：

| 优化 | 说明 |
| :--- | :--- |
| **TileSize 转换** | 世界坐标 ↔ 瓦片坐标自动换算，支持非 1:1 比例 |
| **零分配 A\*** | AStar Finder 内部复用 Node 和 OpenSet 缓冲区，benchmark 显示 0 B/op |
| **LOS Smoothing** | `FindSmoothPath` 基于 Bresenham 直线可视性消除冗余路径点，开销仅 ~25% |
| **可插拔 Finder** | 通过 `WithOrthogonalFinder()` 可替换底层寻路算法 |

#### Waypoint Graph

航点图寻路，基于预计算节点连通性。适用于静态路网场景。

| 优化 | 说明 |
| :--- | :--- |
| **空间哈希索引** | cellSize=64 网格分区，`FindClosest`/`FindClosestWithLOS` 降至 O(1)~O(k) |
| **ConnectNearby 优化** | 从 O(n²) 降至 O(n×k)，仅检查邻近 cell 内的节点对 |
| **LOS 边缘过滤** | 节点连接自动检测障碍物遮挡，仅保留可视连接 |
| **EdgeState 系统** | 支持 Static / Dynamic / Null 三种边状态，可动态阻塞 |

## 压力测试

```bash
# 各算法在 100×100 网格上的压力测试
go test -v -run TestOrthogonalStress ./grid/

# 500×500 大地图压力测试
go test -v -run TestOrthogonalStress_DenseGrid ./grid/

# 六边形网格压力测试
go test -v -run TestHexStress ./grid/

# 交错网格压力测试
go test -v -run TestStaggeredStress ./grid/
```

## 基准测试

```bash
# 跨算法、多规模性能对比（含内存分配）
go test -bench=BenchmarkOrthogonalAlloc -benchmem ./grid/

# 100×100 各算法基准
go test -bench=BenchmarkOrthogonal100 -benchmem ./grid/

# 500×500 大地图基准
go test -bench=BenchmarkOrthogonal500 -benchmem ./grid/

# HPA* 专项基准
go test -bench=. -benchmem ./hpa/

# Waypoint 基准（寻路 + 空间索引）
go test -bench=. -benchmem ./waypoint/

# 运行全部测试
go test ./... -count=1 -vet=all
```

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

### HPA\* 使用示例

```go
import (
    "github.com/actfuns/pathfinding/grid"
    "github.com/actfuns/pathfinding/hpa"
)

// 创建大地图
matrix := generateGrid(500, 500, 0.1)
g := grid.NewOrthogonalGrid(matrix)

// 创建 HPA* finder
f := hpa.NewHPAFinder(
    hpa.WithChunkSize(32),
    hpa.WithStringPulling(true),
)
f.Build(g) // 预构建层级图

// 寻路
path := f.FindPath(0, 0, 499, 499, g)
```

### Waypoint 使用示例

```go
import "github.com/actfuns/pathfinding/waypoint"

g := waypoint.NewWaypointGraph()
a := g.AddNode(0, 0)
b := g.AddNode(10, 0)
c := g.AddNode(10, 10)

// 手动连接
a.Connect(b, waypoint.EdgeStateStatic)
b.Connect(c)

// 或自动连接（距离 + LOS 检测）
grid := walkableGrid(20, 20)
g.ConnectNearby(15, grid)

// 寻路
f := waypoint.NewWaypointFinder(g)
path := f.FindPath(0, 0, 10, 10, grid)
```

## 设计原则

- **零分配热路径** — A\* 和 HPA\* 在预热后达到 0 B/op，减少 GC 压力
- **接口统一** — 所有算法实现 `finder.Finder` 接口，可互换
- **可插拔** — `OrthogonalGrid.WithOrthogonalFinder()` 允许运行时替换寻路算法
- **Option 模式** — 所有配置项通过函数选项安全设置

## 文档

- [Finder API](finder/finder.go) — 所有寻路算法的通用接口
- [Grid](finder/grid.go) — 网格表示和地形定义
- [网格实现](grid/) — 正交、六边形、交错三种网格类型
- [HPA\*](hpa/) — 层级寻路（Portal Compression + String Pulling + Path Cache）
- [Waypoint](waypoint/) — 航点图寻路（空间索引 + LOS 过滤）

## 许可

MIT License — 详见 [LICENSE](LICENSE) 文件。