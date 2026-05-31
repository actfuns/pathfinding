package pathfinding

// BiBreadthFirstFinder is a bidirectional BFS pathfinder.
type BiBreadthFirstFinder struct {
	DiagonalMovement DiagonalMovement
}

// NewBiBreadthFirstFinder creates a new BiBreadthFirstFinder.
func NewBiBreadthFirstFinder(opt *FinderOptions) *BiBreadthFirstFinder {
	f := &BiBreadthFirstFinder{
		DiagonalMovement: DiagonalNever,
	}
	if opt != nil {
		if opt.DiagonalMovement != 0 {
			f.DiagonalMovement = opt.DiagonalMovement
		} else if opt.AllowDiagonal {
			if opt.DontCrossCorners {
				f.DiagonalMovement = DiagonalOnlyWhenNoObstacles
			} else {
				f.DiagonalMovement = DiagonalIfAtMostOneObstacle
			}
		}
	}
	return f
}

// FindPath finds a path using bidirectional BFS.
func (f *BiBreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid *Grid) [][2]int {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	startOpenList := make([]*Node, 0, 64)
	endOpenList := make([]*Node, 0, 64)

	startOpenList = append(startOpenList, startNode)
	startNode.opened = 1
	startNode.parent = nil
	startNode.by = BY_START

	endOpenList = append(endOpenList, endNode)
	endNode.opened = 1
	endNode.parent = nil
	endNode.by = BY_END

	for len(startOpenList) > 0 && len(endOpenList) > 0 {
		// Expand start side
		node := startOpenList[0]
		startOpenList = startOpenList[1:]
		node.closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.closed {
				continue
			}
			if neighbor.opened != 0 {
				if neighbor.by == BY_END {
					return BiBacktrace(node, neighbor)
				}
				continue
			}
			startOpenList = append(startOpenList, neighbor)
			neighbor.parent = node
			neighbor.opened = 1
			neighbor.by = BY_START
		}

		// Expand end side
		node = endOpenList[0]
		endOpenList = endOpenList[1:]
		node.closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.closed {
				continue
			}
			if neighbor.opened != 0 {
				if neighbor.by == BY_START {
					return BiBacktrace(neighbor, node)
				}
				continue
			}
			endOpenList = append(endOpenList, neighbor)
			neighbor.parent = node
			neighbor.opened = 1
			neighbor.by = BY_END
		}
	}

	return nil
}
