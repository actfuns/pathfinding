# Pathfinding

A comprehensive pathfinding library for Go, supporting multiple grid types and search algorithms.

## Features

### Pathfinding Algorithms

| Algorithm | Bidirectional | Description |
|-----------|:-------------:|-------------|
| **A\*** | ✅ | Classic heuristic-based shortest path |
| **Dijkstra** | ✅ | Weighted shortest path (no heuristic) |
| **Best-First Search** | ✅ | Greedy best-first search |
| **Breadth-First Search** | ✅ | Unweighted traversal |
| **IDA\*** | ❌ | Memory-limited iterative deepening A* |
| **JPS (Jump Point Search)** | ❌ | Symmetry-breaking speedup for uniform-cost grids |

### Grid Types

- **Orthogonal** — Standard square grid
- **Hexagonal** — Hex tile grid (axial coordinates)
- **Staggered** — Staggered/offset tile grid

### Advanced Systems

- **HPA\* (Hierarchical Pathfinding A\*)** — Fast pathfinding over large maps using abstraction
- **Waypoint Graph** — Graph-based pathfinding with precomputed node connectivity

## Installation

```bash
go get github.com/actfuns/pathfinding
```

## Quick Start

```go
package main

import (
    "fmt"
    "github.com/actfuns/pathfinding/grid"
)

func main() {
    // Define terrain: 0 = walkable, non-zero = obstacle
    terrain := [][]int{
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0},
    }

    // Create an orthogonal grid (default tile size: 1×1 world unit)
    g := grid.NewOrthogonalGrid(terrain)

    // Find a path in world coordinates
    path := g.FindPath(0, 0, 4, 4)
    fmt.Println("Path:", path)

    // Or with path smoothing
    // smooth := g.FindSmoothPath(0, 0, 4, 4)

    // Modify terrain at runtime
    g.SetWalkableAt(2, 2, false)
}
```

## Documentation

- [Finder API](finder/finder.go) — Common interface for all pathfinding algorithms
- [Grid](finder/grid.go) — Grid representation and terrain definitions
- [Grid implementations](grid/) — Orthogonal, hexagonal, and staggered grid types
- [HPA\*](hpa/) — Hierarchical pathfinding
- [Waypoint](waypoint/) — Waypoint graph pathfinding

## License

MIT License — see the [LICENSE](LICENSE) file for details.