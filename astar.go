package pathfinding

// AStarFinder is an implementation of the A* pathfinding algorithm.
type AStarFinder struct {
	Heuristic        HeuristicFunc
	Weight           float64
	DiagonalMovement DiagonalMovement
}

// NewAStarFinder creates a new AStarFinder with default options.
func NewAStarFinder(opt *FinderOptions) *AStarFinder {
	f := &AStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
		DiagonalMovement: DiagonalNever,
	}
	if opt != nil {
		f.applyOptions(opt)
	}
	// When diagonal movement is allowed, use octile heuristic
	if f.DiagonalMovement != DiagonalNever {
		f.Heuristic = Octile
	}
	return f
}

func (f *AStarFinder) applyOptions(opt *FinderOptions) {
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

// FindPath finds a path from (startX, startY) to (endX, endY) on the grid.
func (f *AStarFinder) FindPath(startX, startY, endX, endY int, grid *Grid) [][2]int {
	openList := NewMinHeap(func(a, b *Node) bool {
		return a.f < b.f
	})
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	startNode.g = 0
	startNode.f = 0
	openList.Push(startNode)
	startNode.opened = 1

	for !openList.Empty() {
		node := openList.Pop()
		node.closed = true

		if node == endNode {
			return Backtrace(endNode)
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)
		for _, neighbor := range neighbors {
			if neighbor.closed {
				continue
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
					openList.Push(neighbor)
					neighbor.opened = 1
				} else {
					openList.UpdateItem(neighbor)
				}
			}
		}
	}

	return nil
}
