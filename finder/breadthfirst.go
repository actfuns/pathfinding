package finder

import (
	"github.com/parasol/pathfinding/core"
)

// BreadthFirstFinder is a Breadth-First-Search pathfinder.
type BreadthFirstFinder struct {
	DiagonalMovement core.DiagonalMovement
}

// NewBreadthFirstFinder creates a new BreadthFirstFinder.
func NewBreadthFirstFinder(opt *FinderOptions) *BreadthFirstFinder {
	f := &BreadthFirstFinder{
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

// FindPath finds a path using BFS.
func (f *BreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid *core.Grid) [][2]int {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	openList := make([]*core.Node, 0, 64)
	openList = append(openList, startNode)
	startNode.Opened = 1

	for len(openList) > 0 {
		node := openList[0]
		openList = openList[1:]
		node.Closed = true

		if node == endNode {
			return core.Backtrace(endNode)
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.Closed || neighbor.Opened != 0 {
				continue
			}
			openList = append(openList, neighbor)
			neighbor.Opened = 1
			neighbor.Parent = node
		}
	}

	return nil
}
