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
    terrain := [][]int32{
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0},
    }

    // 创建正交网格（默认瓦片大小 1×1 世界单位）
    g := grid.NewOrthogonalGrid(terrain)

    // 在世界坐标系中寻路
    path, err := g.FindPathWorld(0, 0, 4, 4, nil)
    if err != nil {
        panic(err) // 搜索没能完成（例如超过时间限制）
    }
    if len(path) == 0 {
        fmt.Println("没有可达路径")
        return
    }
    fmt.Println("路径:", path)

    // 单线程时可显式转为可写视图；并发搜索期间不能写。
    g.(grid.MutableGrid).SetWalkableAt(2, 2, false)
}
```

也可创建全空网格：

```go
m := grid.NewMatrix(100, 100)    // 100×100 全空
g := grid.NewOrthogonalGrid(m)
```

### 瓦片权重（Tile Weight Cost）

v0.5.0+ 支持每个瓦片的移动成本权重。A*、Dijkstra、IDA* 会逐格累加目标瓦片的 `Weight` 值。

> **JPS / JPS+ 假设所有步进成本相同，会忽略瓦片权重。** 需要按权重选择最优路径时请使用 `A*` 或 `Dijkstra`。

```go
g := grid.NewOrthogonalGrid(matrix)
m := g.(grid.MutableGrid) // 单线程零拷贝可写视图

// 设置地形权重：道路(0.3)，草地(1.0)，沼泽(5.0)，岩浆(10.0)
m.SetWeightAt(2, 3, 5.0)
m.SetWeightAt(3, 3, 5.0)
m.SetWeightAt(5, 0, 0.3)  // 低成本道路

f := finder.NewAStarFinder()
path, _ := f.FindPath(0, 0, 7, 7, g, nil)
// A* 会自动选择总成本最低的路径，而非步骤最少的路径
```

默认权重为 `1.0`。`Weight` 必须是**有限正数**（`> 0`）；传入 `0`、负数、`NaN` 或 `Inf` 会 panic。不可通行请用 `SetWalkableAt(false)`，不要用 0 权重表示墙。

成本感知算法是 `A*`、`Dijkstra`、`IDA*`；`JPS` / `JPS+` 忽略权重，`BFS` 和 `Best-First Search` 也不使用成本。

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
    grid.WithFinder(finder.NewJumpPointFinder()),
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

- **瓦片坐标查询** — Width, Height, IsInside, IsWalkableAt, GetNodeAt, ObstacleCount
- **瓦片权重** — GetWeightAt / GetWeightAtWorld（写权重使用 `MutableGrid.SetWeightAt`）
- **流场群体寻路** — flowfield.New / GetDirection（O(1) 多人查表，v0.5.0+）
- **世界坐标转换** — WorldToTile, TileToWorld, IsWalkableAtWorld
- **最近可通行点** — FindNearestWalkable, FindNearestWalkableWorld
- **寻路** — `FindPath` / `FindPathWorld`，返回 `([][2]int32, error)`（可用 `WithSmoothBresenham()` 等选项做路径平滑）
- **渲染** — RenderSVG(cfg, paths...)（权重着色 + 多路径 + 图例，v0.5.0+）

### 可写网格与并发

`NewOrthogonalGrid` / `NewHexGrid` / `NewStaggeredGrid` 返回只读的 `grid.Grid` 接口，不暴露 Setter。

- **单线程修改同一份存储**：显式断言为 `grid.MutableGrid`：

```go
g := grid.NewOrthogonalGrid(matrix)
g.(grid.MutableGrid).SetWalkableAt(1, 2, false)
```

- **只做只读快照**：用 `Clone`。无状态 Finder 自动共享；有状态 Finder 返回 `grid.ErrFinderRequired`：

```go
g2, err := g.Clone()
```

- **并发搜索期间修改**：用 `CloneWith` 复制一次，批量修改后得到新快照：

```go
g2, err := g.CloneWith(func(m grid.MutableGrid) {
    m.SetWalkableAt(1, 2, false)
    m.SetWeightAt(3, 4, 5)
})
if err != nil {
    // 原 Finder 有状态且回调没有 SetFinder 时返回 grid.ErrFinderRequired
}
```

`MutableGrid` 的断言是零拷贝的，调用方必须保证没有并发读/搜索；`CloneWith` 返回的新网格与旧网格完全隔离。无状态 Finder 会被自动共享；JPS+ / HPA / Waypoint 等有状态 Finder 不会复制，回调必须 `SetFinder` 换成针对新网格准备好的 Finder，否则返回 `grid.ErrFinderRequired`。可写接口包含 `SetWalkableAt`、`SetWeightAt`、`SetWalkableAtWorld`、`SetWeightAtWorld` 和 `SetFinder`。

## 寻路结果与错误

`FindPath` / `FindPathWorld` 统一返回 `([][2]int32, error)`，三种结果形态：

| 返回 | 含义 | 处理方式 |
| :--- | :--- | :--- |
| `path, nil` | 找到路径 | 正常使用 |
| `nil, nil` | 搜索完成，**确实没有路** | 换目标点或视为关卡问题 |
| `nil, err` | 请求无效，或搜索**没能完成** | 调用方错误要修参数；超时/池忙可重试；不支持的网格要换算法 |

`err` 目前有这些来源：

- `finder.ErrTimeLimit` — 超过 `WithTimeLimit` 设定的时限（可重试）
- `finder.ErrOutOfBounds` — 端点坐标不在网格内（调用方的问题）
- `finder.ErrNotWalkable` — **终点**瓦片不可走（调用方的问题）；起点不可走不报错，仍允许寻路走出
- `finder.ErrUnsupportedGrid` — 该算法不支持当前网格拓扑（如 JPS/JPS+、HPA 用于 hex/staggered）
- `finder.ErrPrecomputedMismatch` — JPS+ 预计算数据与当前网格尺寸不一致
- `finder.ErrSearchBusy` — 等待空闲搜索槽超时，搜索没有执行，可稍后重试
- 状态/scratch 池满时 `FindPath` 会阻塞等待，不返回错误（见[并发寻路](#并发寻路)）

关键点：**「没有路」不是错误**。目标不可达是算法的正常结论，所以绝大多数调用方只判断 `len(path)` 就够了，现有代码不检查 error 也依然正确：

```go
path, err := g.FindPath(0, 0, 63, 63, nil)
if err != nil {
    if errors.Is(err, finder.ErrOutOfBounds) || errors.Is(err, finder.ErrNotWalkable) {
        // 请求本身有问题：修起点/终点
    }
    // 超时 / 池忙 —— 换个帧重试
}
if len(path) == 0 {
    // 真的没路
}
```

需要区分二者的场景：单位会因为「这一帧太忙」就**永久放弃一条真实存在的路线**。用 error 把这两种状态分开，才能让前者重试、后者换目标。

## 并发寻路

搜索状态已经从 `finder.Node` 拆到 per-FindPath 的 scratch，由有界池按需创建：默认上限是 `GOMAXPROCS`，不会预先创建这么多实例。因此：

- **同一个 Finder、同一个只读 Grid** 可以并发调用 `FindPath` / `FindPathWorld`。
- 返回的 `[][2]int32` / `[][2]float32` 是独立结果，归调用方所有，不需要在下一次调用前拷贝。
- **不能**在搜索期间通过 `MutableGrid` 调用 `SetWalkableAt` / `SetWeightAt`，也不能并发修改 Finder 配置；需要并发更新时使用 `CloneWith` 生成新快照。
- JPS+ 的 `Precompute`、HPA 的 `Build`、waypoint 图的增删边都必须在并发搜索开始前完成；它们与搜索并发不安全。
- 每个 `FindPath` 占用一个状态槽；池会先复用空闲槽，没有空闲且未到上限时新建，达到上限后阻塞等待别人释放。内存随实际并发量增长。
- 默认上限是 `runtime.GOMAXPROCS(0)`，可在启动时调用 `finder.SetMaxConcurrentSearches(n)` 调整；调低不会取消正在运行的搜索。
- 等待上限默认是 `finder.DefaultSearchWait`（1 秒），可用 `finder.SetMaxSearchWait(d)` 调整；设为 `<= 0` 表示不等待，直接返回 `finder.ErrSearchBusy`。

```go
// 现在直接并发即可，不需要 Pool/Proxy。
g := grid.NewOrthogonalGrid(terrain, grid.WithTileSize(32, 32))
f := finder.NewAStarFinder(finder.WithDiagonal(finder.DiagonalAlways))

var wg sync.WaitGroup
for i := 0; i < 8; i++ {
    wg.Add(1)
    go func() {
        defer wg.Done()
        path, err := f.FindPath(0, 0, 200, 200, g, nil)
        _ = path
        _ = err
    }()
}
wg.Wait()
```

> 并发读网格、并发使用同一个 Finder 是安全的；写网格、改 Finder 配置、Precompute/Build 仍需要外部同步。

### 路径缓冲

tile 寻路的最后一参是 `[][2]int32` 缓冲；只有 world 需要 `PathBuff`：

```go
// Finder / Grid tile 路径
FindPath(..., grid Grid, dst [][2]int32) ([][2]int32, error)

// Grid world 路径
FindPathWorld(wx1, wy1, wx2, wy2 float32, buff *grid.PathBuff) ([][2]float32, error)
```

```go
type PathBuff struct {
    Tile  [][2]int32
    World [][2]float32
}

func NewPathBuff(tileCap, worldCap int) *PathBuff
func (b *PathBuff) Reset()
```

用法：

```go
tile := make([][2]int32, 0, 512)
path, err := f.FindPath(sx, sy, ex, ey, grid, tile[:0])

buff := grid.NewPathBuff(512, 512)
wpath, err := g.FindPathWorld(wx1, wy1, wx2, wy2, buff)

// 普通调用
path, err = f.FindPath(sx, sy, ex, ey, grid, nil)
wpath, err = g.FindPathWorld(wx1, wy1, wx2, wy2, nil)
```

- `dst == nil` / `buff == nil`：普通分配
- `dst != nil`：复用 tile 路径
- `buff.Tile`：复用 tile scratch
- `buff.World`：复用 world 输出
- 容量不够时自动扩容，并回写 `PathBuff` 的 slice 头
- 返回 slice 归调用方所有；`dst` / `PathBuff` 不能被并发调用共享
- 搜索状态和临时缓冲会复用。分配表现取决于具体 Finder、输出缓冲容量和搜索状态池是否已热身
- Waypoint 已使用 scratch pool，传 `dst` 预热后可 0 alloc；HPA 仍会分配内部 map/scratch；Flowfield 的 `FindPath` 也接收 `dst`
- Flowfield 构建完成后，`GetDirection` / `GetCost` / `FindPath` 可以并发读；`Reset` / `ResetMulti` 仍不能与读取并发

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
| **HPA\*** | ❌ | 层级寻路，分块抽象加速大地图（仅正交、等成本） |
| **Flow Field** | ❌ | 流场群体寻路，一次 Dijkstra 所有人 O(1) 查方向 |

### JPS 变体

根据对角线移动规则提供四个优化变体：

| 变体 | 对角线规则 | 适用场景 |
| :--- | :--- | :--- |
| **Never** | 禁止对角线 | 四方向移动 |
| **Always** | 始终允许对角线 | 开阔地形 |
| **NoObstacles** | 无障碍时允许对角线 | 半开阔地形 |
| **AtMostOne** | 最多一个障碍时允许对角线 | 复杂障碍布局 |

> JPS / JPS+ 仅适用于**正交网格**（`OrthogonalGrid`）和**等成本网格**（所有瓦片 `Weight == 1`）。hex/staggered 会返回 `finder.ErrUnsupportedGrid`；瓦片权重会被忽略，加权地图请使用 `A*` 或 `Dijkstra`。

### JPS+ (Jump Point Search Plus)

JPS+ 是 JPS 的预计算优化变体，用 O(1) 跳点距离表替换递归 `jump()` 扫描：

| 特性 | 说明 |
| :--- | :--- |
| **Precompute** | `DiagonalAlways` 按方向扫描一次生成跳表，查询时结合终点确定跳点。其他移动模式采用原有跳表和单步搜索 |
| **导出和加载** | `PrecomputedData()` / `LoadPrecomputed()` 保存和读取跳表。新版附带校验和及障碍布局指纹。缺少布局指纹的旧表加载后使用在线 JPS，重新预计算可恢复查表 |
| **地图变更** | 修改可走性后重新预计算。布局与跳表不匹配时，查询会返回 `finder.ErrPrecomputedMismatch` |
| **Adaptive** | 显式启用 `finder.WithAdaptive()` 后，障碍密度达到 10% 时改用 A* |

适用于反复查询的静态、等成本正交地图。选型时可以用实际地图和查询分布，同时测量预计算时间、表大小和查询耗时。

```go
import (
    "github.com/actfuns/pathfinding/finder"
    "github.com/actfuns/pathfinding/grid"
)

matrix := [][]int32{{0, 0, 0}, {0, 1, 0}, {0, 0, 0}}
g := grid.NewOrthogonalGrid(matrix)

f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
if err := f.Precompute(g); err != nil {
    // ErrUnsupportedGrid: JPS+ 只支持正交网格
}

path, _ := f.FindPath(0, 0, 2, 2, g, nil)

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
if err := f.Build(g); err != nil {
    // ErrUnsupportedGrid: HPA 只支持正交网格
}

path, _ := f.FindPath(0, 0, 499, 499, g, nil)
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
path, _ := f.FindPath(0, 0, 10, 10, grid, nil)
```

### OrthogonalGrid 分配与路径平滑

`OrthogonalGrid` 对 A\* 寻路做了极致的内存优化：

| 优化 | 说明 |
| :--- | :--- |
| **TileSize 转换** | 世界坐标 ↔ 瓦片坐标自动换算，支持非 1:1 比例 |
| **零分配 A\*** | AStar Finder 内部复用 Node 和 OpenSet 缓冲区，benchmark 显示 0 B/op |
| **LOS Smoothing** | `WithSmoothBresenham()` 等选项基于 Bresenham 直线可视性自动消除冗余路径点，开销仅 ~25% |
| **可插拔 Finder** | 通过 `WithFinder()` 可替换底层寻路算法 |
| **世界坐标路径** | `FindPathWorld` 使用配置的 Finder 搜索，适用的地形权重和对角规则由 Finder 决定。启用平滑时，随后按几何视线合并路径点 |

## 性能基准

以下是 **Intel i5-13400** 上的历史基准，供理解测试场景和数量级。当前版本请用同一环境、相同查询集重新对照。测试覆盖三种障碍密度（固定种子，可复现）：

- **Open (0%)** — 全空网格
- **Sparse (10%)** — 稀疏障碍
- **Dense (20%)** — 密集障碍

运行命令：`go test -bench=BenchmarkCompare -benchmem ./finder/`

> **数值取 8~10 次重复运行的最小值**，而非中位数。CPU 基准只会被噪声推高、不会推低，因此 minimum 是最稳健的统计量 —— 用中位数会把噪声当成信号，实测曾因此得出「A\* 慢了 6%」的错误结论（真实是最快快 7%）。
>
> `alloc/op` 同样需要足够多的迭代次数才能摊平预热分配：低迭代次数下会看到虚假的 1~2 allocs/op。

| 规模 | 密度 | 算法 | 时间/op | 内存/op | 分配/op |
| :--- | :--- | :--- | ---: | ---: | ---: |
| 100×100 | Open | **A\*** | 189 µs | 25 B | 0 |
| | | **JPS** | 140 µs | 92 B | 3 |
| | | **JPS+** | **0.67 µs** | 28 B | 1 |
| | Sparse | **A\*** | 343 µs | 25 B | 0 |
| | | **JPS** | 43.7 µs | 12.5 KB | 392 |
| | | **JPS+** | **20.0 µs** | 41 B | 1 |
| | Dense | **A\*** | 165 µs | 17 B | 0 |
| | | **JPS** | 29.0 µs | 12.9 KB | 375 |
| | | **JPS+** | **21.1 µs** | 41 B | 1 |
| 200×200 | Open | **A\*** | 1.40 ms | 203 B | 0 |
| | | **JPS** | 1.10 ms | 101 B | 3 |
| | | **JPS+** | **19.6 µs** | 59 B | 1 |
| | Sparse | **A\*** | 858 µs | 107 B | 0 |
| | | **JPS** | 71.7 µs | 25.2 KB | 795 |
| | | **JPS+** | **50.4 µs** | 80 B | 1 |
| | Dense | **A\*** | 828 µs | 107 B | 0 |
| | | **JPS** | 62.1 µs | 27.7 KB | 815 |
| | | **JPS+** | **58.1 µs** | 80 B | 1 |

关键发现：
- **开阔地形 (0%)** — JPS+ 利用预计算跳点表直接跳到目标，100×100 仅 **0.67µs**，比 A\* 快 **282×**
- **有障碍时** — JPS 和 JPS+ 表现接近，100×100 比 A\* 快 **8~16×**
- **A\* 低分配** — 内部复用搜索 scratch；返回路径由调用方拥有，每轮约 1 次分配
- **JPS+ 低分配** — 内联 Bresenham，复用 scratch；路径与展开结果归调用方

### 网格构造

节点存储为连续数组（`nodes []finder.Node`），整个网格仅一次分配，而不是每瓦片一次。200×200、20% 障碍：

| 网格类型 | 时间/op | 内存/op | **分配/op** |
| :--- | ---: | ---: | ---: |
| Orthogonal | 1.62 ms | 4.80 MB | **7** |
| Staggered | 1.40 ms | 4.80 MB | **7** |
| Hex | 1.87 ms | 4.80 MB | **7** |

改造前为 **40,006 次分配 / 5.45 MB**。分配次数下降 **99.98%**，对 GC 压力影响显著；并发搜索时每个 `FindPath` 复用独立 scratch。

### Waypoint 寻路

| 基准 | 时间/op | 内存/op | 分配/op |
| :--- | ---: | ---: | ---: |
| FindPath（预热后） | 178 µs | 7 B | 0 |
| ConnectNearby（676 节点） | 2.44 ms | 244,768 B | 6,901 |

### 各规模对比

同一张 10% 障碍地图（种子 3，强制起终点可通行），HPA\* 使用 `WithChunkSize(32) + WithStringPulling(true)`：

| 规模 | A\* | JPS | JPS+ | HPA\* |
| :--- | ---: | ---: | ---: | ---: |
| 100×100 | 364 µs | 34.9 µs | **25.4 µs** | 167 µs |
| 200×200 | 977 µs | 81.6 µs | **67.4 µs** | 314 µs |
| 500×500 | 3.06 ms | **297 µs** | **239 µs** | 2.08 ms |

结论：
- **JPS+ 在所有规模上最快**，500×500 仍只要 239 µs
- **HPA\* 的收益高度依赖 chunk 大小** — 500×500 下 chunk=32 比 A\* 仅快 **1.5×**，而 chunk=16 可达 **3.3×**（939 µs）
- **大地图上 HPA\* 并非数量级优势**，地图越小越划算；瓶颈在 chunk 内 A\* 的展开量

> ⚠️ 以上数字对障碍布局非常敏感。不同随机种子下 500×500 的 A\* 可相差 10 倍以上，跨机器比较时请用同一份地图生成器复现。

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

### Unreleased — 寻路一致性与 JPS+ 跳表

- 三种内置网格的 `FindPathWorld` 都会调用配置的 Finder，包括无障碍地图。调用方可能收到更多路径点。显式启用的平滑仍在搜索后处理。
- A* 和 IDA* 的默认启发式考虑低于 1 的地形权重。双向 A* 和双向 Dijkstra 按正向边计费，并在已知路径达到停止下界后返回。
- 自定义网格可通过 `finder.SymmetricNeighborGrid` 声明对称邻接。未声明对称的网格使用单向搜索，保留单向边的语义。
- JPS+ 跳表与障碍布局绑定，新格式带校验和。缺少布局指纹的旧表仍可加载，查询会使用在线 JPS，重新运行 `Precompute` 可恢复查表加速。

### Unreleased — 状态外置与直接并发
- `Node` 只保留 `X/Y/Walkable/Weight`；搜索状态改为 per-FindPath scratch，`Parent`/`HeapIndex` 使用 flat index
- `Finder.FindPath` 接收 `[][2]int32` 输出缓冲；`Grid.FindPathWorld` 接收 `*grid.PathBuff`（Tile + World）；核心 finder 可做到预热后 0 alloc/op
- 新增 `ErrSearchBusy` 与 `SetMaxSearchWait`：槽位等待超时返回错误，避免无限阻塞
- 同一 Finder、同一只读 Grid 支持并发 `FindPath` / `FindPathWorld`；返回路径独立，归调用方所有
- 移除 `grid.Pool` / `Proxy` / `PoolManager` / `Lease`；并发由有界状态/scratch 池限制，按需创建到 `GOMAXPROCS` 上限（可用 `SetMaxConcurrentSearches` 调整），未达上限先新建，达到后阻塞等待
- JPS/JPS+ 运行时参数改为 per-call，预计算表只读
- 修复回溯结果未写回缓冲导致的重复分配，以及池化状态被 heap/openList 间接持有的问题

BREAKING: 移除 `Node` 搜索字段/`ResetSearch`、`Backtrace`/`BiBacktrace`/`NewMinHeap`、grid Pool API；JPS+ `Precompute`、HPA `Build` 不可与搜索并发

### Unreleased — int32/float32 全链路迁移
- tile 坐标、路径、网格尺寸与矩阵统一为 `int32` / `[][2]int32` / `[][]int32`
- 权重、代价、启发式与 `G/H/F` 统一为 `float32`；`TimeLimit` 仍为 `float64`
- `StepCost`、`HeuristicFunc` 签名同步迁移；内部 slice 索引仅在边界转换为 `int`

### Unreleased — 只读 Grid 与显式可变视图
- `NewOrthogonalGrid` / `NewHexGrid` / `NewStaggeredGrid` 返回只读 `grid.Grid`
- Setter 移入 `grid.MutableGrid`；单线程可通过类型断言零拷贝访问
- `Grid.Clone() (Grid, error)`：只读快照；有状态 Finder 返回 `ErrFinderRequired`
- 新增 `Grid.CloneWith(func(MutableGrid)) (Grid, error)`：一次复制后批量修改并返回新快照；有状态 Finder 必须显式替换

### v0.9.0 — 并发寻路 + 寻路返回 error
- **破坏性变更**：`finder.Finder.FindPath`、`grid.Grid.FindPath` / `FindPathWorld`、`hpa` / `waypoint` / `flowfield.Field.FindPath` 改为返回 `([][2]int, error)`
- `grid.Pool` 只保留 `Acquire` 一个取实例入口,自带超时上限（默认 `DefaultAcquireTimeout`）；`WithCapacity` 默认取 CPU 核数
- `PoolManager.Acquire` / `Reset` 对 nil 工厂返回 `ErrNilFactory` 而非 panic；`Reset` 不返回 bool
- 新增 `finder.ErrOutOfBounds`：**所有 9 个 finder 及三种网格**的端点越界从 panic 改为返回错误（原先直接 `GetNodeAt` 索引越界崩溃，而 `FindPathWorld` 却返回 nil，行为不一致）
- 三态语义：`(path, nil)` 找到 / `(nil, nil)` 确实没路 / `(nil, err)` 未能完成
- 新增 `finder.ErrTimeLimit`，**修复 IDA\* 超过 `TimeLimit` 后死循环的 bug**（此前永久不返回）
- `grid.Pool`：唯一的 `Acquire` 入口自带超时上限，`WithAcquireTimeout` 可调，`Stats` 提供阻塞统计
- **修复**：工厂 panic 会永久泄漏 capacity 名额，导致 `Acquire` 死锁
- 网格节点存储改为连续数组，构造分配从 40,006 次降到 7 次（三种网格类型）
- A\* 搜索因连续布局提速约 7%（min-of-10 实测）

### v0.9.0 之前 — 并发寻路：网格实例池
- `grid.PoolManager` / `Pool` / `Proxy` / `Lease` / `Factory`
- 多场景共用一张地图时并发寻路：寻路独占租借实例，只读走常驻 Base 实例
- 场景引用计数，最后一个场景释放后自动回收池
- `Reset(key, factory)` 只换地图数据：池与引用计数不变，活跃 `Proxy` 继续有效且自动切到新地图；`Proxy` 本身只读
- 释放后的 Proxy 调用任何方法直接 panic，不静默返回旧数据（`Reset` 不会让 Proxy 失效）
- 结果缓冲复用，寻路热路径仅剩 1 次分配（48 B，来自 `Lease`）
- 网格节点存储改为连续数组，构造分配从 40,006 次降到 7 次（三种网格类型）
- 修正文档中已不存在的 API 名（`WithOrthogonalFinder` → `WithFinder` 等）

### v0.8.x — 网格 API 完善与世界坐标寻路
- `grid.FindPathWorld` 世界坐标寻路
- `TileWidth` / `TileHeight` 瓦片尺寸查询
- `HasLineOfSightWorld` 世界坐标视线检测
- Supercover 直线检测算法与 `WithSmoothSupercover` 平滑
- SVG 渲染逻辑提取至独立文件，统一路径标记绘制

### v0.7.x — 接口整理与 gridutil 工具包
- 新增 `gridutil` 包：随机瓦片 / LOS / 距离 / 划线 / 范围
- `Grid` 接口新增 `TileIndex` / `TileXY`
- 补齐 `FindPath` / `FindSmoothPath` 瓦片版
- `finder` 接口方法移至 `finder` 包，`Clone` 返回类型调整
- 修复 SVG 渲染坐标系翻转

### v0.6.0 — Flow Field 流场群体寻路
- `flowfield/` 包：New/NewMulti 单目标/多目标建场
- Reset/ResetMulti 切换目标，复用内存
- GetDirection O(1) 方向查询（0.5ns）
- GetCost/FindPath 调试支持
- WithDiagonal 对角线支持
- 瓦片权重自动纳入
- examples/flowfield/ 多单位平滑移动示例

### v0.5.x — Tile Weight Cost 权重系统 + SVG 着色渲染
- `finder.Node.Weight` 字段，A*/BiA*/IDA*/JPS/JPS+ 步进成本 × tile.Weight
- `grid.MutableGrid.SetWeightAt` / `grid.Grid.GetWeightAt`，三层网格实现
- `RenderSVG(cfg, paths...)` 权重着色 + 多路径 + 右侧图例
- `SVGOpts` 自定义颜色映射和图例标签
- `DefaultSVGOpts` 全局默认配置
- `examples/` 目录（basic / weighted / svg）
- godoc 注释补全

### v0.4.0
- JPS+/HPA* 性能优化、Bresenham 内联、全局原子计数器

## 设计原则

- **复用搜索 scratch** — heap/neighbor/path 缓冲按搜索复用；返回路径独立分配，支持直接并发
- **接口统一** — 所有算法实现 `finder.Finder` 接口，可互换
- **可插拔** — `grid.WithFinder()` 允许替换底层寻路算法
- **Option 模式** — 所有配置项通过函数选项安全设置
- **接口分离** — `finder.Grid` 只暴露寻路所需方法，`grid.Grid` 提供完整的消费者接口
- **直接并发** — 只读网格 + 独立搜索 scratch；写网格、改配置、Precompute/Build 仍需外部同步

## 文档

- [Finder API](finder/finder.go) — 所有寻路算法的通用接口
- [Grid](finder/grid.go) — 寻路算法所需的网格接口
- [Grid 实现](grid/) — 正交、六边形、交错三种网格类型
- [并发寻路](finder/state_pool.go) — per-FindPath 状态槽与并发约束
- [HPA\*](hpa/) — 层级寻路（Portal Compression + String Pulling + Path Cache）
- [Waypoint](waypoint/) — 航点图寻路（空间索引 + LOS 过滤）
- [Examples](examples/) — 可直接运行的示例（basic/weighted/svg/flowfield）

## 许可

MIT License — 详见 [LICENSE](LICENSE) 文件。
