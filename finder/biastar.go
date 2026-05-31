package finder

import (
	"github.com/actfuns/navpath/core"
)

// BiAStarFinder is a bidirectional A* pathfinder.
type BiAStarFinder struct {
	Heuristic        core.HeuristicFunc
	Weight           float64
	DiagonalMovement core.DiagonalMovement
}

// NewBiAStarFinder creates a new BiAStarFinder.
func NewBiAStarFinder(opt *FinderOptions) *BiAStarFinder {
	f := &BiAStarFinder{
		Heuristic:        core.Manhattan,
		Weight:           1,
		DiagonalMovement: core.DiagonalNever,
	}
	if opt != nil {
		if opt.DiagonalMovement != 0 {
			f.DiagonalMovement = opt.DiagonalMovement
		} else if opt.AllowDiagonal {
			if opt.DontCrossCorners {
				f.DiagonalMovement = core.DiagonalOnlyWhenNoObstacles
			} else {
				f.DiagonalMovement = core.DiagonalIfAtMostOneObstacle
			}
		}
		if opt.Heuristic != nil {
			f.Heuristic = opt.Heuristic
		}
		if opt.Weight > 0 {
			f.Weight = opt.Weight
		}
	}
	if f.DiagonalMovement != core.DiagonalNever {
		f.Heuristic = core.Octile
	}
	return f
}

// FindPath finds a path using bidirectional A*.
func (f *BiAStarFinder) FindPath(startX, startY, endX, endY int, grid *core.Grid) [][2]int {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	cmp := func(a, b *core.Node) bool { return a.F < b.F }
	startOpenList := core.NewMinHeap(cmp)
	endOpenList := core.NewMinHeap(cmp)

	startNode.G = 0
	startNode.F = 0
	startOpenList.Push(startNode)
	startNode.Opened = BY_START

	endNode.G = 0
	endNode.F = 0
	endOpenList.Push(endNode)
	endNode.Opened = BY_END

	for !startOpenList.Empty() && !endOpenList.Empty() {
		// Expand from start side
		node := startOpenList.Pop()
		node.Closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened == BY_END {
				return core.BiBacktrace(node, neighbor)
			}

			x, y := neighbor.X, neighbor.Y
			var ng float64
			if x-node.X == 0 || y-node.Y == 0 {
				ng = node.G + 1
			} else {
				ng = node.G + core.SQRT2
			}

			if neighbor.Opened == 0 || ng < neighbor.G {
				neighbor.G = ng
				if neighbor.Opened == 0 {
					neighbor.H = f.Weight * f.Heuristic(float64(core.AbsInt(x-endX)), float64(core.AbsInt(y-endY)))
				}
				neighbor.F = neighbor.G + neighbor.H
				neighbor.Parent = node

				if neighbor.Opened == 0 {
					startOpenList.Push(neighbor)
					neighbor.Opened = BY_START
				} else {
					startOpenList.UpdateItem(neighbor)
				}
			}
		}

		// Expand from end side
		node = endOpenList.Pop()
		node.Closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened == BY_START {
				return core.BiBacktrace(neighbor, node)
			}

			x, y := neighbor.X, neighbor.Y
			var ng float64
			if x-node.X == 0 || y-node.Y == 0 {
				ng = node.G + 1
			} else {
				ng = node.G + core.SQRT2
			}

			if neighbor.Opened == 0 || ng < neighbor.G {
				neighbor.G = ng
				if neighbor.Opened == 0 {
					neighbor.H = f.Weight * f.Heuristic(float64(core.AbsInt(x-startX)), float64(core.AbsInt(y-startY)))
				}
				neighbor.F = neighbor.G + neighbor.H
				neighbor.Parent = node

				if neighbor.Opened == 0 {
					endOpenList.Push(neighbor)
					neighbor.Opened = BY_END
				} else {
					endOpenList.UpdateItem(neighbor)
				}
			}
		}
	}

	return nil
}
