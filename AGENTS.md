# AGENTS.md

本仓库是一个 Go 寻路库：`github.com/actfuns/pathfinding`，Go 版本见 `go.mod`。
修改代码前优先阅读 `README.md`、`finder/finder.go`、`finder/grid.go` 和对应测试。

## 常用命令

```bash
# 基础测试
go test ./...

# 并发/内存安全检查
go test -race ./...

# 32 位兼容性
GOARCH=386 go test ./...

# CI 使用：Linux 跑 race + stress，Windows 跑 stress
go test -tags stress ./...
go test -race -tags stress ./...
go vet -tags stress ./...

# 性能基准
go test -bench=BenchmarkCompare -benchmem ./finder/
go test -bench=BenchmarkOrthogonalAlloc -benchmem ./grid/
go test -bench=. -benchmem ./hpa/
go test -bench=. -benchmem ./waypoint/
```

`stress` 构建标签保护大网格压力测试；不要把它加入普通测试的默认路径。

## 目录职责

- `finder/`：Finder 接口、搜索算法、搜索状态池、JPS/JPS+ 跳表、通用工具。
- `grid/`：Orthogonal / Hex / Staggered 网格、只读 Grid、MutableGrid、快照、权重、世界坐标路径和 SVG。
- `gridutil/`：LOS、随机地图、平滑等通用网格工具。
- `hpa/`：HPA*，有状态 Finder，搜索前必须 `Build`。
- `waypoint/`：航点图寻路，有状态 Finder。
- `flowfield/`：流场，一次构图、多单位查询。
- `examples/`：可运行示例，改动公共 API 时同步检查。
- `.github/workflows/ci.yml`：CI 命令和平台矩阵。

## 类型与 API 约定

- 瓦片坐标、尺寸、路径使用 `int32` / `[][2]int32`。
- 权重、代价、启发式、`G/H/F` 使用 `float32`。
- `TimeLimit` 使用 `float64`。
- 搜索代际 `SearchID` 当前是 `int64`；不要为了缩字段随意改成 `uint32`。
- `finder.Finder.FindPath` 和 `grid.Grid.FindPath` 返回 `([][2]int32, error)`；
  `grid.Grid.FindPathWorld` 返回 `([][2]float32, error)`。
- 路径结果有三态：
  - `path, nil`：找到路径
  - `nil, nil`：搜索完成但确实没有路
  - `nil, err`：请求无效或搜索未完成，可重试/修参数
- `ErrNotWalkable` 只针对终点；起点不可走仍允许搜索走出。
- 不要为“没有路”返回 error。

## 并发模型

- 只读 Grid 可以被并发搜索；`MutableGrid` 是零拷贝写视图。
- 搜索期间不能通过 `MutableGrid` 改地形/权重，也不能修改 Finder 配置。
- 需要并发修改时使用 `Clone` 生成新快照。
- 无状态 Finder 可以在只读网格上并发复用；JPS+ / HPA / Waypoint 是有状态 Finder：
  - `Precompute` / `Build` / 图增删边必须在并发搜索开始前完成。
  - `Clone` 不会自动复制有状态 Finder，回调里必须 `SetFinder`，否则返回 `grid.ErrFinderRequired`。
- 搜索状态由有界池管理，默认上限是 `runtime.GOMAXPROCS(0)`，
  默认等待时间是 `finder.DefaultSearchWait`（1 秒），超时返回 `finder.ErrSearchBusy`。
- 并发测试会临时提高测试期搜索池容量；不要依赖默认容量来跑高并发测试。

## Grid / 快照

- 构造器返回只读 `grid.Grid`，写入使用 `grid.MutableGrid` 断言。
- 快照 API：

```go
copy, err := g.Clone(nil)                    // 无编辑副本
copy, err := g.Clone(func(m grid.MutableGrid) {
    m.SetWalkableAt(x, y, false)
})
```

- 旧的 `Clone()` / `CloneWith()` 已不再作为当前 API；不要新增二者。
- `Clone` 回调不得保留传入的 `MutableGrid`。
- 修改可走性时必须维持 fingerprint / obstacleCount / weight bounds 的一致性。
- `FindPathWorld` 始终调用配置的 Finder，包括全空地图；不会走旧的两端点捷径。
  - 起点/终点保留调用方传入的原始 world 坐标。
  - 中间点使用瓦片中心。
  - `PathBuff.Tile` 是平滑后的 tile 路径，`PathBuff.World` 是 world 输出。
  - 全空地图也可能有多于两个路径点。

## Finder / JPS+ 约定

- 搜索状态和 scratch 属于单次 `FindPath`，不要重新加回 Finder 的可变字段。
- JPS / JPS+ 只支持正交、等成本、静态网格。
- JPS / JPS+ 忽略瓦片权重；加权地图使用 A* / Dijkstra。
- JPS+ 的 canonical 跳表支持四种对角线模式：
  `DiagonalNever`、`DiagonalOnlyWhenNoObstacles`、
  `DiagonalIfAtMostOneObstacle`、`DiagonalAlways`。
- 跳表必须记录对角线模式；不能跨模式复用。模式不匹配返回 `ErrPrecomputedMismatch`。
- JPS+ 的预计算表与地图 walkability 绑定；修改网格后必须重新 `Precompute`。
- 新表包含 fingerprint / checksum。缺少 fingerprint 的旧表允许加载，但查询会退回在线 JPS。
- 自定义 Grid 若实现 `finder.WalkabilityFingerprintGrid`，可以获得 O(1) 校验；
  没有实现时不要在每次查询里全图扫描，应退回在线 JPS。
- 跳表格式/模式位属于兼容性敏感区域；修改前先看 `finder/jps_plus_codec.go` 的测试和 fuzz。
- `HasSymmetricNeighbors` 决定双向搜索是否启用；没有声明对称的 Grid 必须使用单向搜索。

## 测试要求

提交前至少运行：

```bash
gofmt -w <changed-go-files>
go test ./...
go test -race ./...
GOARCH=386 go test ./...
go vet ./...
```

涉及并发、搜索池、网格写入、JPS+ 表或双向搜索时，再运行：

```bash
go test -tags stress ./...
go test -race -tags stress ./...
```

测试路径正确性时不要只比较长度或耗时，应验证：

- 起点和终点
- 每一步都是 `GetNeighbors` 允许的合法边
- 目标格可走
- 路径代价与 A* / Dijkstra 参考结果一致
- 不可达结果与参考算法一致

随机测试使用固定 seed，保证可复现。JPS+ 表序列化、legacy、损坏数据和模式不匹配已有专门测试，
不要用临时 benchmark 替换这些测试。

## 代码风格

- 遵循 Go 常规格式；提交前运行 `gofmt`。
- 热路径优先复用 `searchStates` / scratch；不要引入每步分配。
- 不要为了理论上的无锁性能重写状态池，先用 mutex/block profile 证明瓶颈。
- 不要在热路径引入全图扫描、接口断言风暴或额外 map 分配。
- 公共 API 变更必须同步 README、godoc 和版本历史。
- 不新增依赖，除非仓库已有同类依赖且确实必要。
- 不要提交依赖 `/tmp` 地图文件或外部环境的 benchmark/test 文件。
- 文件权限保持 `100644`；不要提交意外的 `100755` 变更。

## Git 工作流

- 除非用户明确要求，不要 commit、push、rebase 或打 tag。
- 提交前检查：

```bash
git status --short
git diff
git diff --cached
git log --oneline -5
```

- 只暂存本次任务的代码和测试；不要把其他并发改动、临时 benchmark、模式变更混入提交。
- 提交信息遵循仓库已有的 conventional 风格，例如：

```text
perf(jps+): enable canonical tables for all diagonal modes
fix(grid): honor finder rules in world paths
test: size concurrent search pool for stress tests
```

- 已发布的 tag 不要移动；需要发布修复时使用新 tag。
- 用户说“先不要提交/不要打 tag”时，保持工作区改动，不要自行提交或推送。

## 已知关注点

这些不是当前阻塞项，但修改相关代码时要留意：

- HPA 的 `pathCache` 目前没有容量淘汰策略，长期服务查询分布大时可能持续增长。
- 部分网格/状态代码使用 `width*height` 或 `index` 的 `int32` 中间计算；
  新增超大网格支持时需要统一溢出保护。
- Hex / Staggered 当前声明邻接对称；修改拓扑或 stagger 配置时需重新验证双向搜索。
- 四种 JPS+ 模式已有正确性对照；性能基准主要覆盖开阔地图，
  障碍密集/真实地图的性能需要单独测量。
- `Finder` godoc 中“递归调用会永久等待”的表述应按实际 `SetMaxSearchWait` 语义理解/修正。
