# Pathfinding

一个全面的 Go 寻路库，支持多种网格类型和搜索算法。

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

    // 在运行时修改地形
    g.SetWalkableAt(2, 2, false)
}
```

也可创建全空网格：

```go
m := grid.NewMatrix(100, 100)    // 100×100 全空
g := grid.NewOrthogonalGrid(m)
```

### 瓦片权重（Tile Weight Cost）

v0.5.0+ 支持每个瓦片的移动成本权重。A*、Dijkstra、IDA*、JPS、JPS+ 在计算步进成本时自动乘以目标瓦片的 Weight 值。

```go
g := grid.NewOrthogonalGrid(matrix)

// 设置地形权重：道路(0.3)，草地(1.0)，沼泽(5.0)，岩浆(10.0)
g.SetWeightAt(2, 3, 5.0)
g.SetWeightAt(3, 3, 5.0)
g.SetWeightAt(5, 0, 0.3)  // 低成本道路

f := finder.NewAStarFinder()
path := f.FindPath(0, 0, 7, 7, g)
// A* 会自动选择总成本最低的路径，而非步骤最少的路径
```

默认权重为 `1.0`，设为 `0` 表示不可通行。所有成本感知算法（A*、Dijkstra、IDA*、JPS、JPS+）都支持。BFS 和 Best-First Search 不使用成本计算，因此不受权重影响。

### SVG 可视化

`RenderSVG` 根据瓦片权重自动着色，并显示图例：

```go
// 默认着色（nil = 使用 DefaultSVGOpts）
svg := g.RenderSVG(nil, path)

// 自定义颜色映射
svg := g.RenderSVG(&grid.SVGOpts{
    WeightColors: [][2]string{
        {"0.3", "#bbdefb"}, // ≤0.3 蓝色（道路）
        {"1.0", "#c8e6c9"}, // ≤1.0 绿色（草地）
        {"5.0", "#ffcc80"}, // ≤5.0 橙色（沼泽）
        {"+Inf", "#ef5350"},// >5.0 红色（极端）
    },
    LegendEntries: [][2]string{
        {"#bbdefb", "Road"},
        {"#c8e6c9", "Grass"},
        {"#ffcc80", "Swamp"},
        {"#555555", "Wall"},
    },
}, path)
```

支持多路径叠加（第一条蓝色实线，第二条红色虚线）：

```go
svg := g.RenderSVG(nil, pathWithWeights, pathWithoutWeights)
```

环境变量 `NAVPATH_DUMP_SVG=1` 可在测试时自动输出 SVG 到 `grid/testdata/svg/`。

### 使用其他寻路算法

```go
import "github.com/actfuns/pathfinding/finder"

// 替换默认的 A* 为 JPS
g := grid.NewOrthogonalGrid(terrain,
    grid.WithOrthogonalFinder(finder.NewJumpPointFinder()),
)
```

## 网格类型

| 类型 | 说明 |
| :--- | :--- |
| **正交网格 (Orthogonal)** | 标准方格网格，支持自定义 TileSize 转换世界坐标 |
| **六边形网格 (Hexagonal)** | 六边形瓦片网格（轴向坐标），支持 flat-top / pointy-top |
| **交错网格 (Staggered)** | 45 度等距/交错瓦片网格（钻石形瓦片） |

### Grid 接口

所有网格类型实现统一的 `grid.Grid` 接口：

- **瓦片坐标查询** — Width, Height, IsInside, IsWalkableAt, SetWalkableAt, GetNodeAt, ObstacleCount
- **瓦片权重** — SetWeightAt, GetWeightAt（v0.5.0+）
- **流场群体寻路** — flowfield.New / GetDirection（O(1) 多人查表，v0.5.0+）
- **世界坐标转换** — WorldToTile, TileToWorld, IsWalkableAtWorld, SetWalkableAtWorld
- **最近可通行点** — FindNearestWalkable, FindNearestWalkableTile
- **寻路** — FindPath, FindSmoothPath, SmoothenTilePath
- **渲染** — RenderSVG(cfg, paths...)（权重着色 + 多路径 + 图例，v0.5.0+）

## 寻路算法

| 算法 | 双向 | 说明 |
| :--- | :---: | :--- |
| **A\*** | ✅ | 经典启发式最短路径 |
| **Dijkstra** | ✅ | 加权最短路径（无启发式） |
| **Best-First Search** | ✅ | 贪心最佳优先搜索 |
| **Breadth-First Search** | ✅ | 广度优先遍历 |
| **IDA\*** | ❌ | 迭代加深 A\*，内存占用极低 |
| **JPS (Jump Point Search)** | ❌ | 跳点搜索，利用网格对称性加速 |
| **JPS+ (Jump Point Search Plus)** | ❌ | JPS 优化变体，Precompute 跳点表，O(1) 查表替换递归扫描 |
| **HPA\*** | ❌ | 层级寻路，分块抽象加速大地图 |
| **Flow Field** | ❌ | 流场群体寻路，一次 Dijkstra 所有人 O(1) 查方向 |

### JPS 变体

根据对角线移动规则提供四个优化变体：

| 变体 | 对角线规则 | 适用场景 |
| :--- | :--- | :--- |
| **Never** | 禁止对角线 | 四方向移动 |
| **Always** | 始终允许对角线 | 开阔地形 |
| **NoObstacles** | 无障碍时允许对角线 | 半开阔地形 |
| **AtMostOne** | 最多一个障碍时允许对角线 | 复杂障碍布局 |

### JPS+ (Jump Point Search Plus)

JPS+ 是 JPS 的预计算优化变体，用 O(1) 跳点距离表替换递归 `jump()` 扫描：

| 特性 | 说明 |
| :--- | :--- |
| **Precompute** | 两阶段预计算：Phase 1 检测 Primary Jump Points，Phase 2 计算 8 方向距离表 |
| **export/import** | `PrecomputedData()` / `LoadPrecomputed()` 支持烘焙数据导出为文件，服务端启动直接加载 |
| **searchSeq 全局唯一** | 所有 finder 的 `searchSeq` 通过全局原子计数器初始化，不同 finder 可安全共享同一 grid |

适用场景：**障碍物密集的静态小地图**（如游戏房间内布局固定的障碍地图）。开阔地形下利用预计算距离表可直接跳跃到目标，反而是 JPS+ 最快的场景（100×100 仅 0.6µs）。

```go
import (
    "github.com/actfuns/pathfinding/finder"
    "github.com/actfuns/pathfinding/grid"
)

matrix := [][]int{{0, 0, 0}, {0, 1, 0}, {0, 0, 0}}
g := grid.NewOrthogonalGrid(matrix)

f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
f.Precompute(g)

path := f.FindPath(0, 0, 2, 2, g)

// 导出烘焙数据供后续启动直接加载
data, _ := f.PrecomputedData()
// os.WriteFile("map_data.bin", data, 0644)
```

## 高级系统

### HPA\* (Hierarchical Pathfinding A\*)

层级寻路，通过抽象分层加速大地图寻路。核心优化：

| 优化 | 说明 |
| :--- | :--- |
| **Portal Compression (P0)** | 将相邻可通行边缘合并为连续 Portal，减少抽象图节点数 4~8× |
| **String Pulling (P1)** | 对最终路径进行 LOS 平滑，消除层级抽象产生的冗余拐点 |
| **Allocation Optimization (P2)** | 复用内部缓冲区（portalPathBuf、waypointsBuf、waypointsResult），预热后零分配 |
| **Portal Path Cache (P3)** | 缓存 (startChunk, endChunk) → portal 路径，重复查询直接复用，避免抽象层 A\* |

```go
import (
    "github.com/actfuns/pathfinding/grid"
    "github.com/actfuns/pathfinding/hpa"
)

matrix := generateGrid(500, 500, 0.1)
g := grid.NewOrthogonalGrid(matrix)

f := hpa.NewHPAFinder(
    hpa.WithChunkSize(32),
    hpa.WithStringPulling(true),
)
f.Build(g)

path := f.FindPath(0, 0, 499, 499, g)
```

### Waypoint Graph

航点图寻路，基于预计算节点连通性。适用于静态路网场景。

| 优化 | 说明 |
| :--- | :--- |
| **空间哈希索引** | cellSize=64 网格分区，`FindClosest`/`FindClosestWithLOS` 降至 O(1)~O(k) |
| **ConnectNearby 优化** | 从 O(n²) 降至 O(n×k)，仅检查邻近 cell 内的节点对 |
| **LOS 边缘过滤** | 节点连接自动检测障碍物遮挡，仅保留可视连接 |
| **EdgeState 系统** | 支持 Static / Dynamic / Null 三种边状态，可动态阻塞 |

```go
import "github.com/actfuns/pathfinding/waypoint"

g := waypoint.NewWaypointGraph()
a := g.AddNode(0, 0)
b := g.AddNode(10, 0)
c := g.AddNode(10, 10)

a.Connect(b, waypoint.EdgeStateStatic)
b.Connect(c)

f := waypoint.NewWaypointFinder(g)
path := f.FindPath(0, 0, 10, 10, grid)
```

### OrthogonalGrid 分配与路径平滑

`OrthogonalGrid` 对 A\* 寻路做了极致的内存优化：

| 优化 | 说明 |
| :--- | :--- |
| **TileSize 转换** | 世界坐标 ↔ 瓦片坐标自动换算，支持非 1:1 比例 |
| **零分配 A\*** | AStar Finder 内部复用 Node 和 OpenSet 缓冲区，benchmark 显示 0 B/op |
| **LOS Smoothing** | `FindSmoothPath` 基于 Bresenham 直线可视性消除冗余路径点，开销仅 ~25% |
| **可插拔 Finder** | 通过 `WithOrthogonalFinder()` 可替换底层寻路算法 |
| **障碍物计数** | `obstacleCount` 跟踪不可通行瓦片数，全空时 FindPath 直接返回线段，不走算法 |

## 性能基准

以下基准测试在 **Intel i5-13400** 上运行。测试三种障碍密度（固定种子，可复现）：

- **Open (0%)** — 全空网格
- **Sparse (10%)** — 稀疏障碍
- **Dense (20%)** — 密集障碍

运行命令：`go test -bench=BenchmarkCompare -benchmem ./finder/`

| 规模 | 密度 | 算法 | 时间/op | 内存/op | 分配/op |
| :--- | :--- | :--- | ---: | ---: | ---: |
| 100×100 | Open | **A\*** | 150 µs | 0 B | 0 |
| | | **JPS** | 122 µs | 88 B | 3 |
| | | **JPS+** | **0.6 µs** | 24 B | 1 |
| | Sparse | **A\*** | 294 µs | 0 B | 0 |
| | | **JPS** | 29 µs | 12.5 KB | 392 |
| | | **JPS+** | **19 µs** | 24 B | 1 |
| | Dense | **A\*** | 135 µs | 0 B | 0 |
| | | **JPS** | 24 µs | 12.9 KB | 375 |
| | | **JPS+** | **20 µs** | 24 B | 1 |
| 200×200 | Open | **A\*** | 1.10 ms | 0 B | 0 |
| | | **JPS** | 605 µs | 88 B | 3 |
| | | **JPS+** | **19 µs** | 24 B | 1 |
| | Sparse | **A\*** | 667 µs | 0 B | 0 |
| | | **JPS** | 62 µs | 25.2 KB | 795 |
| | | **JPS+** | **45 µs** | 24 B | 1 |
| | Dense | **A\*** | 662 µs | 0 B | 0 |
| | | **JPS** | 54 µs | 27.7 KB | 815 |
| | | **JPS+** | **53 µs** | 24 B | 1 |

关键发现：
- **开阔地形 (0%)** — JPS+ 利用预计算跳点表直接跳到目标，100×100 仅 **0.6µs**，比 A\* 快 **250×**
- **有障碍时** — JPS 和 JPS+ 表现接近，比 A\* 快 3~12×
- **A\* 零分配** — 内部复用 Node/OpenSet 缓冲区，全程 0 B/op
- **JPS+ 近乎零分配** — 内联 Bresenham 消除 ExpandPath 分配，仅返回路径的 1 次分配

### Waypoint 寻路

| 基准 | 时间/op | 内存/op | 分配/op |
| :--- | ---: | ---: | ---: |
| FindPath（预热后） | 139,615 ns | 1 B | 0 |
| ConnectNearby（676 节点） | 1,508,913 ns | 244,768 B | 6,901 |

### 各规模对比

| 规模 | A\* | JPS | JPS+ | HPA\* |
| :--- | ---: | ---: | ---: | ---: |
| 100×100 (10% obs) | 294 µs | 29 µs | **19 µs** | 2,911 ns |
| 200×200 (10% obs) | 667 µs | 62 µs | **45 µs** | 3,782 ns |
| 500×500 (10% obs) | 25 ms | 46 ms | — | **35 µs** |

HPA\* 在 500×500 上相比 A\* 加速约 **730×**。

## 命令

```bash
# 运行全部测试（压力测试需 -tags stress）
go test ./... -count=1 -vet=all

# 压力测试（构建标签保护，默认不运行）
go test -tags stress -v -run Stress ./grid/

# 基准测试（A* / JPS / JPS+ 对比）
go test -bench=BenchmarkCompare -benchmem ./finder/
# 基准测试（Orthogonal / Hex / Staggered 网格）
go test -bench=BenchmarkOrthogonalAlloc -benchmem ./grid/
go test -bench=BenchmarkOrthogonal100 -benchmem ./grid/
# 基准测试（HPA* / Waypoint）
go test -bench=. -benchmem ./hpa/
go test -bench=. -benchmem ./waypoint/

# 生成 SVG 可视化（有权重场景自动着色 + 图例）
NAVPATH_DUMP_SVG=1 go test -run "TestOrthogonalSVG|TestHexSVG|TestStaggeredSVG" ./grid/
ls grid/testdata/svg/*weighted*.svg
```

## 示例

`examples/` 目录包含可直接运行的示例：

```bash
# 基本 A* 寻路
go run examples/basic/

# 权重地形对比（显示权重如何影响路径选择）
go run examples/weighted/

# SVG 可视化输出
go run examples/svg/ | head -5

# 流场群体寻路
go run examples/flowfield/
```

## 版本历史

### v0.5.x — Tile Weight Cost 权重系统 + SVG 着色渲染
- `finder.Node.Weight` 字段，A*/BiA*/IDA*/JPS/JPS+ 步进成本 × tile.Weight
- `grid.Grid.SetWeightAt` / `GetWeightAt`，三层网格实现
- `RenderSVG(cfg, paths...)` 权重着色 + 多路径 + 右侧图例
- `SVGOpts` 自定义颜色映射和图例标签
- `DefaultSVGOpts` 全局默认配置
- `examples/` 目录（basic / weighted / svg）
- godoc 注释补全

### v0.4.0
- JPS+/HPA* 性能优化、Bresenham 内联、全局原子计数器

## 设计原则

- **零分配热路径** — A\* 和 HPA\* 在预热后达到 0 B/op，减少 GC 压力
- **接口统一** — 所有算法实现 `finder.Finder` 接口，可互换
- **可插拔** — `OrthogonalGrid.WithOrthogonalFinder()` 允许运行时替换寻路算法
- **Option 模式** — 所有配置项通过函数选项安全设置
- **接口分离** — `finder.Grid` 只暴露寻路所需方法，`grid.Grid` 提供完整的消费者接口

## 文档

- [Finder API](finder/finder.go) — 所有寻路算法的通用接口
- [Grid](finder/grid.go) — 寻路算法所需的网格接口
- [Grid 实现](grid/) — 正交、六边形、交错三种网格类型
- [HPA\*](hpa/) — 层级寻路（Portal Compression + String Pulling + Path Cache）
- [Waypoint](waypoint/) — 航点图寻路（空间索引 + LOS 过滤）
- [Examples](examples/) — 可直接运行的示例（basic/weighted/svg/flowfield）

## 许可

MIT License — 详见 [LICENSE](LICENSE) 文件。