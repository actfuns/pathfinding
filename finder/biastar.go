package finder

// BiAStarFinder is a bidirectional A* pathfinder.
type BiAStarFinder struct {
	Heuristic        HeuristicFunc
	Weight           float64
	DiagonalMovement DiagonalMovement
}

// NewBiAStarFinder creates a new BiAStarFinder.
func NewBiAStarFinder(opts ...Option) *BiAStarFinder {
	opt := ApplyOptions(opts)
	f := &BiAStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
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

// FindPath finds a path using bidirectional A*.
func (f *BiAStarFinder) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	cmp := func(a, b *Node) bool { return a.F < b.F }
	startOpenList := NewMinHeap(cmp)
	endOpenList := NewMinHeap(cmp)

	startNode.G = 0
	startNode.F = 0
	startOpenList.Push(startNode)
	startNode.Opened = BY_START

	endNode.G = 0
	endNode.F = 0
	endOpenList.Push(endNode)
	endNode.Opened = BY_END

	neighborBuf := make([]*Node, 0, 8)

	for !startOpenList.Empty() && !endOpenList.Empty() {
		node := startOpenList.Pop()
		node.Closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened == BY_END {
				return BiBacktrace(node, neighbor)
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
					startOpenList.Push(neighbor)
					neighbor.Opened = BY_START
				} else {
					startOpenList.UpdateItem(neighbor)
				}
			}
		}

		node = endOpenList.Pop()
		node.Closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			if neighbor.Closed {
				continue
			}
			if neighbor.Opened == BY_START {
				return BiBacktrace(neighbor, node)
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
					neighbor.H = f.Weight * f.Heuristic(float64(AbsInt(x-startX)), float64(AbsInt(y-startY)))
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
