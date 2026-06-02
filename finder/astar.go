package finder

// AStarFinder is an implementation of the A* pathfinding algorithm.
type AStarFinder struct {
	Heuristic        HeuristicFunc
	Weight           float64
	DiagonalMovement DiagonalMovement

	searchSeq   int // incremented each FindPath call for node state reset
	neighborBuf []*Node
	heapSlice   []*Node
	pathBuf     [][2]int
}

// NewAStarFinder creates a new AStarFinder with default options.
func NewAStarFinder(opts ...Option) *AStarFinder {
	opt := ApplyOptions(opts)
	f := &AStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
		DiagonalMovement: DiagonalNever,
		neighborBuf:      make([]*Node, 0, 8),
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
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	}
	if opt.Weight > 0 {
		f.Weight = opt.Weight
	}
	if f.DiagonalMovement != DiagonalNever {
		f.Heuristic = Octile
	}
	return f
}

// FindPath finds a path from (startX, startY) to (endX, endY) on the grid.
func (f *AStarFinder) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	f.searchSeq++
	openList := &MinHeap{nodes: f.heapSlice[:0]}
	defer func() { f.heapSlice = openList.nodes[:0] }()
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	startNode.ResetSearch(f.searchSeq)
	endNode.ResetSearch(f.searchSeq)
	startNode.G = 0
	startNode.F = 0
	openList.Push(startNode)
	startNode.Opened = 1

	neighborBuf := f.neighborBuf[:0]

	for !openList.Empty() {
		node := openList.Pop()
		node.Closed = true

		if node == endNode {
			f.pathBuf = f.pathBuf[:0]
			for n := endNode; n != nil; n = n.Parent {
				f.pathBuf = append(f.pathBuf, [2]int{n.X, n.Y})
			}
			for i, j := 0, len(f.pathBuf)-1; i < j; i, j = i+1, j-1 {
				f.pathBuf[i], f.pathBuf[j] = f.pathBuf[j], f.pathBuf[i]
			}
			return f.pathBuf
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			neighbor.ResetSearch(f.searchSeq)
			if neighbor.Closed {
				continue
			}

			x, y := neighbor.X, neighbor.Y
			var ng float64
			if x-node.X == 0 || y-node.Y == 0 {
				ng = node.G + 1
			} else {
				ng = node.G + SQRT2
			}

			if neighbor.Opened == 0 || ng < neighbor.G {
				neighbor.G = ng
				if neighbor.Opened == 0 {
					neighbor.H = f.Weight * f.Heuristic(float64(AbsInt(x-endX)), float64(AbsInt(y-endY)))
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
