package finder

import (
	"github.com/parasol/pathfinding/core"
)

// BiBreadthFirstFinder is a bidirectional BFS pathfinder.
type BiBreadthFirstFinder struct {
	DiagonalMovement core.DiagonalMovement
}

// NewBiBreadthFirstFinder creates a new BiBreadthFirstFinder.
func NewBiBreadthFirstFinder(opt *FinderOptions) *BiBreadthFirstFinder {
	f := &BiBreadthFirstFinder{
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
	}
	return f
}

// FindPath finds a path using bidirectional BFS.
func (f *BiBreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid *core.Grid) [][2]int {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	startOpenList := make([]*core.Node, 0, 64)
	endOpenList := make([]*core.Node, 0, 64)

	startOpenList = append(startOpenList, startNode)
	startNode.Opened = 1
	startNode.Parent = nil
	startNode.By = BY_START

	endOpenList = append(endOpenList, endNode)
	endNode.Opened = 1
	endNode.Parent = nil
	endNode.By = BY_END

	for len(startOpenList) > 0 && len(endOpenList) > 0 {
		// Expand start side
		node := startOpenList[0]
		startOpenList = startOpenList[1:]
		node.Closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened != 0 {
				if neighbor.By == BY_END {
					return core.BiBacktrace(node, neighbor)
				}
				continue
			}
			startOpenList = append(startOpenList, neighbor)
			neighbor.Parent = node
			neighbor.Opened = 1
			neighbor.By = BY_START
		}

		// Expand end side
		node = endOpenList[0]
		endOpenList = endOpenList[1:]
		node.Closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened != 0 {
				if neighbor.By == BY_START {
					return core.BiBacktrace(neighbor, node)
				}
				continue
			}
			endOpenList = append(endOpenList, neighbor)
			neighbor.Parent = node
			neighbor.Opened = 1
			neighbor.By = BY_END
		}
	}

	return nil
}
