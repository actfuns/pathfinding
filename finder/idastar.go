package finder

import (
	"time"

	"github.com/actfuns/navpath/core"
)

// IDAStarFinder is an Iterative Deepening A* pathfinder.
type IDAStarFinder struct {
	Heuristic        core.HeuristicFunc
	Weight           float64
	DiagonalMovement core.DiagonalMovement
	TrackRecursion   bool
	TimeLimit        float64 // seconds, <= 0 for infinite
}

// NewIDAStarFinder creates a new IDAStarFinder.
func NewIDAStarFinder(opt *FinderOptions) *IDAStarFinder {
	f := &IDAStarFinder{
		Heuristic:        core.Manhattan,
		Weight:           1,
		DiagonalMovement: core.DiagonalNever,
		TrackRecursion:   false,
		TimeLimit:        -1,
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
		if opt.Heuristic != nil {
			f.Heuristic = opt.Heuristic
		}
		if opt.Weight > 0 {
			f.Weight = opt.Weight
		}
		f.TrackRecursion = opt.TrackRecursion
		f.TimeLimit = opt.TimeLimit
	}
	if f.DiagonalMovement != core.DiagonalNever {
		f.Heuristic = core.Octile
	}
	return f
}

// FindPath finds a path using IDA*.
func (f *IDAStarFinder) FindPath(startX, startY, endX, endY int, grid *core.Grid) [][2]int {
	start := grid.GetNodeAt(startX, startY)
	end := grid.GetNodeAt(endX, endY)

	h := func(a, b *core.Node) float64 {
		return f.Heuristic(float64(core.AbsInt(b.X-a.X)), float64(core.AbsInt(b.Y-a.Y)))
	}

	cost := func(a, b *core.Node) float64 {
		if a.X == b.X || a.Y == b.Y {
			return 1
		}
		return core.SQRT2
	}

	startTime := time.Now()
	hasTimeLimit := f.TimeLimit > 0

	var search func(node *core.Node, g, cutoff float64, route [][2]int, depth int) (float64, bool)

	search = func(node *core.Node, g, cutoff float64, route [][2]int, depth int) (float64, bool) {
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
		neighbors := grid.GetNeighbors(node, f.DiagonalMovement)

		for _, neighbor := range neighbors {
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
