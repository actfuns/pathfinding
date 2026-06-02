package finder

import (
	"sync/atomic"
	"time"
)

// IDAStarFinder is an Iterative Deepening A* pathfinder.
type IDAStarFinder struct {
	Heuristic        HeuristicFunc
	Weight           float64
	DiagonalMovement DiagonalMovement
	TrackRecursion   bool
	TimeLimit        float64

	searchSeq   int
	neighborBuf []*Node
}

// NewIDAStarFinder creates a new IDAStarFinder.
func NewIDAStarFinder(opts ...Option) *IDAStarFinder {
	opt := ApplyOptions(opts)
	f := &IDAStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
		DiagonalMovement: DiagonalNever,
		TrackRecursion:   false,
		TimeLimit:        -1,
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
	f.TrackRecursion = opt.TrackRecursion
	f.TimeLimit = opt.TimeLimit
	if f.DiagonalMovement != DiagonalNever {
		f.Heuristic = Octile
	}
	return f
}

// FindPath finds a path using IDA*.
func (f *IDAStarFinder) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	f.searchSeq = int(atomic.AddUint64(&globalSearchSeq, 1))
	start := grid.GetNodeAt(startX, startY)
	end := grid.GetNodeAt(endX, endY)
	start.ResetSearch(f.searchSeq)
	end.ResetSearch(f.searchSeq)

	h := func(a, b *Node) float64 {
		return f.Heuristic(float64(AbsInt(b.X-a.X)), float64(AbsInt(b.Y-a.Y)))
	}

	cost := func(a, b *Node) float64 {
		if a.X == b.X || a.Y == b.Y {
			return 1
		}
		return SQRT2
	}

	startTime := time.Now()
	hasTimeLimit := f.TimeLimit > 0

	neighborBuf := f.neighborBuf[:0]
	var search func(node *Node, g, cutoff float64, route [][2]int, depth int) (float64, bool)

	search = func(node *Node, g, cutoff float64, route [][2]int, depth int) (float64, bool) {
		if hasTimeLimit && time.Since(startTime).Seconds() > f.TimeLimit {
			return 0, false
		}

		fScore := g + h(node, end)*f.Weight
		if fScore > cutoff {
			return fScore, false
		}

		if node == end {
			route[depth] = [2]int{node.X, node.Y}
			return 0, true
		}

		min := float64(0)
		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)

		for _, neighbor := range neighbors {
			neighbor.ResetSearch(f.searchSeq)
			if f.TrackRecursion {
				neighbor.RetainCount++
				if !neighbor.Tested {
					neighbor.Tested = true
				}
			}

			t, found := search(neighbor, g+cost(node, neighbor), cutoff, route, depth+1)

			if found {
				route[depth] = [2]int{node.X, node.Y}
				return 0, true
			}

			if f.TrackRecursion {
				neighbor.RetainCount--
				if neighbor.RetainCount == 0 {
					neighbor.Tested = false
				}
			}

			if min == 0 || t < min {
				min = t
			}
		}

		return min, false
	}

	cutOff := h(start, end)

	for j := 0; ; j++ {
		route := make([][2]int, 512)

		t, found := search(start, 0, cutOff, route, 0)

		if found {
			pathLen := 0
			for i := 0; i < len(route); i++ {
				if route[i] == [2]int{0, 0} {
					if i > 0 {
						break
					}
				} else {
					pathLen = i + 1
				}
			}
			if pathLen == 0 {
				return nil
			}
			path := make([][2]int, pathLen)
			copy(path, route[:pathLen])
			return path
		}

		if t == 0 {
			return nil
		}
		cutOff = t
	}
}
