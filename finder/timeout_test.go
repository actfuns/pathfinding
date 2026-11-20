package finder

import (
	"errors"
	"runtime"
	"testing"
	"time"
)

type timeoutGrid struct {
	nodes []Node
	w, h  int
}

func (g *timeoutGrid) Width() int             { return g.w }
func (g *timeoutGrid) Height() int            { return g.h }
func (g *timeoutGrid) IsInside(x, y int) bool { return x >= 0 && x < g.w && y >= 0 && y < g.h }
func (g *timeoutGrid) IsWalkableAt(x, y int) bool {
	return g.IsInside(x, y)
}
func (g *timeoutGrid) SetWalkableAt(x, y int, v bool) {}
func (g *timeoutGrid) GetNodeAt(x, y int) *Node       { return &g.nodes[y*g.w+x] }
func (g *timeoutGrid) GetNeighbors(n *Node, _ DiagonalMovement, buf []*Node) []*Node {
	out := buf[:0]
	for _, d := range [][2]int{{0, -1}, {1, 0}, {0, 1}, {-1, 0}} {
		x, y := n.X+d[0], n.Y+d[1]
		if g.IsInside(x, y) {
			out = append(out, g.GetNodeAt(x, y))
		}
	}
	return out
}
func (g *timeoutGrid) Clone() Grid { return g }

func TestSearchBusyTimeout(t *testing.T) {
	oldMax := statePool.max
	oldWait := statePool.wait
	defer func() {
		SetMaxConcurrentSearches(oldMax)
		SetMaxSearchWait(oldWait)
	}()

	grid := &timeoutGrid{w: 4, h: 4, nodes: make([]Node, 16)}
	for y := 0; y < 4; y++ {
		for x := 0; x < 4; x++ {
			grid.nodes[y*4+x] = Node{X: x, Y: y, Walkable: true, Weight: 1}
		}
	}

	SetMaxConcurrentSearches(1)
	SetMaxSearchWait(20 * time.Millisecond)
	held, ok := statePool.acquire()
	if !ok {
		t.Fatal("could not acquire held state")
	}
	_, err := NewAStarFinder().FindPath(0, 0, 3, 3, grid, nil)
	statePool.release(held)
	if !errors.Is(err, ErrSearchBusy) {
		t.Fatalf("err = %v, want ErrSearchBusy", err)
	}

	SetMaxSearchWait(DefaultSearchWait)
	SetMaxConcurrentSearches(runtime.GOMAXPROCS(0))
}
