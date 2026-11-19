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
    path, err := g.FindPathWorld(0, 0, 4, 4)
    if err != nil {
        panic(err) // 搜索没能完成（例如超过时间限制）
    }
    if len(path) == 0 {
        fmt.Println("没有可达路径")
        return
    }
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

v0.5.0+ 支持每个瓦片的移动成本权重。A*、Dijkstra、IDA* 会逐格累加目标瓦片的 `Weight` 值。

> **JPS / JPS+ 假设所有步进成本相同，会忽略瓦片权重。** 需要按权重选择最优路径时请使用 `A*` 或 `Dijkstra`。

```go
g := grid.NewOrthogonalGrid(matrix)

// 设置地形权重：道路(0.3)，草地(1.0)，沼泽(5.0)，岩浆(10.0)
g.SetWeightAt(2, 3, 5.0)
g.SetWeightAt(3, 3, 5.0)
g.SetWeightAt(5, 0, 0.3)  // 低成本道路

f := finder.NewAStarFinder()
path, _ := f.FindPath(0, 0, 7, 7, g)
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

- **瓦片坐标查询** — Width, Height, IsInside, IsWalkableAt, SetWalkableAt, GetNodeAt, ObstacleCount
- **瓦片权重** — SetWeightAt, GetWeightAt（v0.5.0+）
- **流场群体寻路** — flowfield.New / GetDirection（O(1) 多人查表，v0.5.0+）
- **世界坐标转换** — WorldToTile, TileToWorld, IsWalkableAtWorld, SetWalkableAtWorld
- **最近可通行点** — FindNearestWalkable, FindNearestWalkableWorld
- **寻路** — `FindPath` / `FindPathWorld`，返回 `([][2]int, error)`（可用 `WithSmoothBresenham()` 等选项做路径平滑）
- **渲染** — RenderSVG(cfg, paths...)（权重着色 + 多路径 + 图例，v0.5.0+）

## 寻路结果与错误

`FindPath` / `FindPathWorld` 统一返回 `([][2]int, error)`，三种结果形态：

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
- `grid.ErrPoolTimeout` / `grid.ErrPoolClosed` — 池层面的失败（见[并发寻路](#并发寻路)）

关键点：**「没有路」不是错误**。目标不可达是算法的正常结论，所以绝大多数调用方只判断 `len(path)` 就够了，现有代码不检查 error 也依然正确：

```go
path, err := g.FindPath(0, 0, 63, 63)
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

### 为什么单个网格不能并发

`OrthogonalGrid` / `HexGrid` / `StaggeredGrid` **不是并发安全的**。一次搜索会改写节点的可搜索状态（`G`、`F`、`Parent`、`Closed`、`Opened`…）并复用内部缓冲区（`pathBuf`、`heapSlice`、`neighborBuf`），而返回值又直接指向那块缓冲区，只在下次调用前有效。因此同一个网格上并发跑两次寻路会互相踩踏、返回错乱的路径。

### 方案：PoolManager 按场景发放代理

多个场景共用同一张地图时，每个场景从 `PoolManager` 取一个 `Proxy`（代理）。代理是**只读视图**：

- **寻路** — 从 `Pool` 租借一个独占实例执行，用完立即归还，跨场景的并发搜索因此永不共享 finder 状态
- **其余读方法** — 转发给池里常驻的只读 `Base` 实例，不需要租借、不会被寻路阻塞
- **修改地图** — 不再通过 `Proxy`；构造新的 `Factory` 后调用 `PoolManager.Reset`

```go
mgr := grid.NewPoolManager()

// 每个场景一个 key（通常是地图名），并且只由该场景自己的 goroutine 驱动
mapProxy, err := mgr.Acquire("town", func() grid.Grid {
    return grid.NewOrthogonalGrid(terrain, grid.WithTileSize(32, 32))
}, grid.WithCapacity(4))
if err != nil {
    return err
}
defer mapProxy.Release()

path, err := mapProxy.FindPathWorld(0, 0, 200, 200)
// err 非 nil 表示搜索未能完成（池忙 / 超过时间限制），而不是"没有路"
```

`Acquire(key, factory, opts...)` 里 `key` 是地图名、`factory` 造实例(必填,为 nil 返回 `ErrNilFactory`),两者都只在池首次创建时读取。`opts` 可调并发上限和超时,都有默认值:

| 选项 | 默认 | 说明 |
| :--- | :--- | :--- |
| `WithCapacity(n)` | `runtime.NumCPU()` | 该地图同时能跑几个搜索。空闲时不占内存(突发结束后会缩回场景数),但每多一个实例就是一整份地图拷贝 |
| `WithAcquireTimeout(d)` | `DefaultAcquireTimeout`(1 秒) | `Acquire` 最多等 d,到期返回 `ErrPoolTimeout`。`<= 0` 表示无限等 |

### 契约

> 一句话总结并发的形状：**并行靠代理，不靠锁。** 多个场景共享同一个 `Pool`（地图数据、实例、可调并发上限），但每个场景拿自己那个 `Proxy`，并且只在自己的 goroutine 上用。

| 约定 | 说明 |
| :--- | :--- |
| **一个 Proxy 一个 goroutine** | **最硬的约束。** Proxy 缓存了读取用的 Base，并持有每个结果的写入缓冲 —— 两个 goroutine 驱动同一个 Proxy 会同时破坏这两样，**静默返回错乱的路径，不会 panic、没有任何报错**。检测代码不存在，也**不应该**加：一次原子操作会让只读路径从 4.2ns 涨到 9.5ns（2.3 倍），换来的是一个你已经能保证的约束。并发请**取多个 Proxy**，那才是 `Pool` 的用途。<br>**安全的**：多个 Proxy、任意多 goroutine ✅　**不安全**：一个 Proxy、多个 goroutine ❌ |
| **工厂必须造完全独立的实例** | 每个实例需要自己的瓦片存储和自己的 Finder。**不要用 `Grid.Clone()`** —— 它复制节点但共享 Finder，两个克隆体并发搜索会损坏 |
| **Proxy 只读** | `Proxy` 不再暴露 `SetWalkableAt` / `SetWeightAt`。要改地图，构造新 `Factory` 后调用 `Reset`，否则租借实例看不到修改。本设计面向静态地图 |
| **必须 Release** | 场景销毁时调用一次。最后一个场景释放后池自动回收；重复调用安全 |
| **校验不靠运气** | `Acquire` / `Reset` 对 nil 工厂返回 `ErrNilFactory`，`Reset` 目标无人持有时返回 `ErrNoPool` —— 不会静默"成功" |
| **Release 后即失效** | 对已释放的 Proxy 调用任何 Proxy 方法会 **panic**，而不是静默返回旧数据（`Reset` 不会让 Proxy 失效）；释放时会清掉 Base/缓冲引用，便于回收 |
| **池满有上限** | `WithCapacity` 是并发上限（默认 CPU 核数），到顶后 `FindPath` 等待实例归还，但不会超过池的获取超时（默认 `DefaultAcquireTimeout`） |
| **容量不影响空闲内存** | 一轮突发结束后池会缩回场景数，所以 `capacity` 只决定峰值，按最繁忙值给即可 |

### Acquire 有超时上限

`Pool.Acquire` 是唯一的取实例入口。池满时它不会无限等 —— 到超时上限就返回 `grid.ErrPoolTimeout`，**即使一个实例都没归还也会醒**。

默认上限 `DefaultAcquireTimeout`（1 秒）。这个值故意设得宽松：500×500 的 A\* 搜索实测约 3 ms，留了约 300 倍余量，慢搜索不会被误伤。它要挡的只是一件事 —— **租借泄漏**（调用方忘记 Release，或 panic 时没走 defer）会导致后来者永久阻塞。

主循环有帧预算时，用 Option 收紧：

```go
pr, err := mgr.Acquire("town", factory,
    grid.WithCapacity(8),
    grid.WithAcquireTimeout(2*time.Millisecond))

path, err := pr.FindPathWorld(0, 0, 200, 200)
switch {
case errors.Is(err, grid.ErrPoolTimeout):
    // 没抢到实例，本帧跳过，下次再试
case err != nil:
    // 搜索没能完成
case len(path) == 0:
    // 真的没路
}
```

`Timeout <= 0` 表示无限等待（等于放弃上限），只能显式要。

### 用统计决定上限该设多少

不必猜。先读 `Stats()`：

```go
s := pr.Stats()
if s.Waits > 0 && s.MaxWait > time.Millisecond {
    log.Printf("town 寻路曾阻塞: %d 次, 累计 %v, 最长 %v",
        s.Waits, s.WaitTotal, s.MaxWait)
}
```

`Stats` 是累计值，取两次快照相减就是单帧增量；`MaxWait` 是历史峰值。**如果 `Waits` 一直是 0，说明 `capacity` 给够了，超时永远触发不到。**

统计只在调用方**真的发生阻塞**时记录，不阻塞的快路径不读时钟。`Stats()` 本身约 18 ns / 0 分配，每帧轮询一次无压力。

### 地图变化：Reset

`Reset` 只换**地图数据**,其它一概不动 —— 池对象不变、场景引用计数不变、已经发出去的 `Proxy` 全部继续有效、**不会 panic**。

```go
// 地图热重载 / 开门 / 换图,随便什么时候调用
if err := mgr.Reset("town", newFactory); err != nil {
    return err
}
// 之前拿到的 pr 照常能用,而且已经在新地图上工作了
```

内部做法是给池换工厂并递增一个 epoch:旧工厂建出来的实例随即作废 —— 空闲的立刻丢弃,在借的等这次搜索跑完再丢 —— 所以**下一次租借就会用新数据重建**。已经开始的搜索会跑完它开始时的那份地图。

```go
pr, _ := mgr.Acquire("town", factory, grid.WithCapacity(4))
pr.FindPathWorld(...)        // 用旧地图
mgr.Reset("town", newFactory)
pr.FindPathWorld(...)        // 用新地图，同一个 pr
```

几点说明：

- **容量和超时属于池、不属于地图**,`Reset` 不改它们。新 key 请在 `Acquire` 时传新的 options。
- 多个场景同时持有时,`Reset` 之后它们**都**切到新地图,不需要谁重新 `Acquire`。
- `Reset` 返回 `ErrNilFactory`(工厂为 nil)或 `ErrNoPool`(该 key 无人持有),不会静默"成功"。

### 返回值生命周期

`Proxy.FindPath` / `FindPathWorld` 的结果指向代理自己的缓冲，**在下一次搜索前有效**。需要长期持有请自行拷贝——这也是它与 `grid.Grid` 其他方法一致的地方。

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
| **Precompute** | 两阶段预计算：Phase 1 检测 Primary Jump Points，Phase 2 计算 8 方向距离表 |
| **export/import** | `PrecomputedData()` / `LoadPrecomputed()` 支持烘焙数据导出为文件，服务端启动直接加载 |
| **searchSeq 全局唯一** | 所有 finder 的 `searchSeq` 通过全局原子计数器初始化，不同 finder 可安全共享同一 grid |
| **Adaptive** | `finder.WithAdaptive()`：障碍密度 ≥10% 时自动改用 A*，避免密集图上的退化 |

适用场景：**障碍物密集的静态小地图**（如游戏房间内布局固定的障碍地图）。开阔地形下利用预计算距离表可直接跳跃到目标，反而是 JPS+ 最快的场景（100×100 仅 0.6µs）。

```go
import (
    "github.com/actfuns/pathfinding/finder"
    "github.com/actfuns/pathfinding/grid"
)

matrix := [][]int{{0, 0, 0}, {0, 1, 0}, {0, 0, 0}}
g := grid.NewOrthogonalGrid(matrix)

f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
if err := f.Precompute(g); err != nil {
    // ErrUnsupportedGrid: JPS+ 只支持正交网格
}

path, _ := f.FindPath(0, 0, 2, 2, g)

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

path, _ := f.FindPath(0, 0, 499, 499, g)
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
path, _ := f.FindPath(0, 0, 10, 10, grid)
```

### OrthogonalGrid 分配与路径平滑

`OrthogonalGrid` 对 A\* 寻路做了极致的内存优化：

| 优化 | 说明 |
| :--- | :--- |
| **TileSize 转换** | 世界坐标 ↔ 瓦片坐标自动换算，支持非 1:1 比例 |
| **零分配 A\*** | AStar Finder 内部复用 Node 和 OpenSet 缓冲区，benchmark 显示 0 B/op |
| **LOS Smoothing** | `WithSmoothBresenham()` 等选项基于 Bresenham 直线可视性自动消除冗余路径点，开销仅 ~25% |
| **可插拔 Finder** | 通过 `WithFinder()` 可替换底层寻路算法 |
| **障碍物计数** | `obstacleCount` 跟踪不可通行瓦片数，全空时 FindPath 直接返回线段，不走算法 |

## 性能基准

以下基准测试在 **Intel i5-13400** 上运行。测试三种障碍密度（固定种子，可复现）：

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
- **A\* 零分配** — 内部复用 Node/OpenSet 缓冲区，全程 0 allocs/op
- **JPS+ 近乎零分配** — 内联 Bresenham 消除 ExpandPath 分配，仅返回路径的 1 次分配

### 网格构造

节点存储为连续数组（`nodes []finder.Node`），整个网格仅一次分配，而不是每瓦片一次。200×200、20% 障碍：

| 网格类型 | 时间/op | 内存/op | **分配/op** |
| :--- | ---: | ---: | ---: |
| Orthogonal | 1.62 ms | 4.80 MB | **7** |
| Staggered | 1.40 ms | 4.80 MB | **7** |
| Hex | 1.87 ms | 4.80 MB | **7** |

改造前为 **40,006 次分配 / 5.45 MB**。分配次数下降 **99.98%**，对 GC 压力影响显著 —— 使用[并发寻路](#并发寻路)时，池内每个实例都受益。

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
- **可插拔** — `grid.WithFinder()` 允许替换底层寻路算法
- **Option 模式** — 所有配置项通过函数选项安全设置
- **接口分离** — `finder.Grid` 只暴露寻路所需方法，`grid.Grid` 提供完整的消费者接口
- **并发靠代理** — 网格本身非并发安全；需要并发时用 `grid.PoolManager` 按场景发放 `Proxy`

## 文档

- [Finder API](finder/finder.go) — 所有寻路算法的通用接口
- [Grid](finder/grid.go) — 寻路算法所需的网格接口
- [Grid 实现](grid/) — 正交、六边形、交错三种网格类型
- [并发寻路](grid/pool_manager.go) — PoolManager / Pool / Proxy，网格实例池与场景生命周期
- [HPA\*](hpa/) — 层级寻路（Portal Compression + String Pulling + Path Cache）
- [Waypoint](waypoint/) — 航点图寻路（空间索引 + LOS 过滤）
- [Examples](examples/) — 可直接运行的示例（basic/weighted/svg/flowfield）

## 许可

MIT License — 详见 [LICENSE](LICENSE) 文件。