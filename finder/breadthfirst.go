package finder

// BreadthFirstFinder is a Breadth-First-Search pathfinder.
type BreadthFirstFinder struct {
	DiagonalMovement DiagonalMovement

	searchSeq int
}

// NewBreadthFirstFinder creates a new BreadthFirstFinder.
func NewBreadthFirstFinder(opts ...Option) *BreadthFirstFinder {
	opt := ApplyOptions(opts)
	f := &BreadthFirstFinder{
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

// FindPath finds a path using BFS.
func (f *BreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	f.searchSeq++
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	startNode.ResetSearch(f.searchSeq)
	endNode.ResetSearch(f.searchSeq)
	openList := make([]*Node, 0, 64)
	openList = append(openList, startNode)
	startNode.Opened = 1

	neighborBuf := make([]*Node, 0, 8)

	for len(openList) > 0 {
		node := openList[0]
		openList = openList[1:]
		node.Closed = true

		if node == endNode {
			return Backtrace(endNode)
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			neighbor.ResetSearch(f.searchSeq)
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
