package finder

// BiBreadthFirstFinder is a bidirectional BFS pathfinder.
type BiBreadthFirstFinder struct {
	DiagonalMovement DiagonalMovement

	searchSeq int
}

// NewBiBreadthFirstFinder creates a new BiBreadthFirstFinder.
func NewBiBreadthFirstFinder(opts ...Option) *BiBreadthFirstFinder {
	opt := ApplyOptions(opts)
	f := &BiBreadthFirstFinder{
		DiagonalMovement: DiagonalNever,
	}
	if opt.DiagonalMovement != 0 {
		f.DiagonalMovement = opt.DiagonalMovement
	} else if opt.AllowDiagonal {
		if opt.DontCrossCorners {
			f.DiagonalMovement = DiagonalOnlyWhenNoObstacles
		} else {
			f.DiagonalMovement = DiagonalIfAtMostOneObstacle
		}
	}
	return f
}

// FindPath finds a path using bidirectional BFS.
func (f *BiBreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	f.searchSeq++
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	startNode.ResetSearch(f.searchSeq)
	endNode.ResetSearch(f.searchSeq)

	const (
		BY_START = 1
		BY_END   = 2
	)

	startOpenList := make([]*Node, 0, 64)
	endOpenList := make([]*Node, 0, 64)

	startOpenList = append(startOpenList, startNode)
	startNode.Opened = 1
	startNode.Parent = nil
	startNode.By = BY_START

	endOpenList = append(endOpenList, endNode)
	endNode.Opened = 1
	endNode.Parent = nil
	endNode.By = BY_END

	neighborBuf := make([]*Node, 0, 8)

	for len(startOpenList) > 0 && len(endOpenList) > 0 {
		node := startOpenList[0]
		startOpenList = startOpenList[1:]
		node.Closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			neighbor.ResetSearch(f.searchSeq)
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened != 0 {
				if neighbor.By == BY_END {
					return BiBacktrace(node, neighbor)
				}
				continue
			}
			startOpenList = append(startOpenList, neighbor)
			neighbor.Parent = node
			neighbor.Opened = 1
			neighbor.By = BY_START
		}

		node = endOpenList[0]
		endOpenList = endOpenList[1:]
		node.Closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			neighbor.ResetSearch(f.searchSeq)
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened != 0 {
				if neighbor.By == BY_START {
					return BiBacktrace(neighbor, node)
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