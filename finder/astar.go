package finder

import (
	"github.com/actfuns/navpath/core"
)

// FinderOptions holds common options for all finders.
type FinderOptions struct {
	AllowDiagonal    bool
	DontCrossCorners bool
	DiagonalMovement core.DiagonalMovement
	Heuristic        core.HeuristicFunc
	Weight           float64
	TrackRecursion   bool
	TimeLimit        float64 // in seconds, <= 0 for infinite
}

// AStarFinder is an implementation of the A* pathfinding algorithm.
type AStarFinder struct {
	Heuristic        core.HeuristicFunc
	Weight           float64
	DiagonalMovement core.DiagonalMovement
}

// NewAStarFinder creates a new AStarFinder with default options.
func NewAStarFinder(opt *FinderOptions) *AStarFinder {
	f := &AStarFinder{
		Heuristic:        core.Manhattan,
		Weight:           1,
		DiagonalMovement: core.DiagonalNever,
	}
	if opt != nil {
		f.applyOptions(opt)
	}
	// When diagonal movement is allowed, use octile heuristic
	if f.DiagonalMovement != core.DiagonalNever {
		f.Heuristic = core.Octile
	}
	return f
}

func (f *AStarFinder) applyOptions(opt *FinderOptions) {
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

// FindPath finds a path from (startX, startY) to (endX, endY) on the grid.
func (f *AStarFinder) FindPath(startX, startY, endX, endY int, grid *core.Grid) [][2]int {
	openList := core.NewMinHeap(func(a, b *core.Node) bool {
		return a.F < b.F
	})
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	startNode.G = 0
	startNode.F = 0
	openList.Push(startNode)
	startNode.Opened = 1

	for !openList.Empty() {
		node := openList.Pop()
		node.Closed = true

		if node == endNode {
			return core.Backtrace(endNode)
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
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
					openList.Push(neighbor)
					neighbor.Opened = 1
				} else {
					openList.UpdateItem(neighbor)
				}
			}
		}
	}

	return nil
}
