package pathfinding

// BiAStarFinder is a bidirectional A* pathfinder.
type BiAStarFinder struct {
	Heuristic        HeuristicFunc
	Weight           float64
	DiagonalMovement DiagonalMovement
}

// NewBiAStarFinder creates a new BiAStarFinder.
func NewBiAStarFinder(opt *FinderOptions) *BiAStarFinder {
	f := &BiAStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
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
		if opt.Heuristic != nil {
			f.Heuristic = opt.Heuristic
		}
		if opt.Weight > 0 {
			f.Weight = opt.Weight
		}
	}
	if f.DiagonalMovement != DiagonalNever {
		f.Heuristic = Octile
	}
	return f
}

// FindPath finds a path using bidirectional A*.
func (f *BiAStarFinder) FindPath(startX, startY, endX, endY int, grid *Grid) [][2]int {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	cmp := func(a, b *Node) bool { return a.f < b.f }
	startOpenList := NewMinHeap(cmp)
	endOpenList := NewMinHeap(cmp)

	startNode.g = 0
	startNode.f = 0
	startOpenList.Push(startNode)
	startNode.opened = BY_START

	endNode.g = 0
	endNode.f = 0
	endOpenList.Push(endNode)
	endNode.opened = BY_END

	for !startOpenList.Empty() && !endOpenList.Empty() {
		// Expand from start side
		node := startOpenList.Pop()
		node.closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.closed {
				continue
			}
			if neighbor.opened == BY_END {
				return BiBacktrace(node, neighbor)
			}

			x, y := neighbor.X, neighbor.Y
			var ng float64
			if x-node.X == 0 || y-node.Y == 0 {
				ng = node.g + 1
			} else {
				ng = node.g + SQRT2
			}

			if neighbor.opened == 0 || ng < neighbor.g {
				neighbor.g = ng
				if neighbor.opened == 0 {
					neighbor.h = f.Weight * f.Heuristic(float64(absInt(x-endX)), float64(absInt(y-endY)))
				}
				neighbor.f = neighbor.g + neighbor.h
				neighbor.parent = node

				if neighbor.opened == 0 {
					startOpenList.Push(neighbor)
					neighbor.opened = BY_START
				} else {
					startOpenList.UpdateItem(neighbor)
				}
			}
		}

		// Expand from end side
		node = endOpenList.Pop()
		node.closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.closed {
				continue
			}
			if neighbor.opened == BY_START {
				return BiBacktrace(neighbor, node)
			}

			x, y := neighbor.X, neighbor.Y
			var ng float64
			if x-node.X == 0 || y-node.Y == 0 {
				ng = node.g + 1
			} else {
				ng = node.g + SQRT2
			}

			if neighbor.opened == 0 || ng < neighbor.g {
				neighbor.g = ng
				if neighbor.opened == 0 {
					neighbor.h = f.Weight * f.Heuristic(float64(absInt(x-startX)), float64(absInt(y-startY)))
				}
				neighbor.f = neighbor.g + neighbor.h
				neighbor.parent = node

				if neighbor.opened == 0 {
					endOpenList.Push(neighbor)
					neighbor.opened = BY_END
				} else {
					endOpenList.UpdateItem(neighbor)
				}
			}
		}
	}

	return nil
}
