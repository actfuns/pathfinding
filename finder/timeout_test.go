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

func (g *timeoutGrid) Width() int32  { return int32(g.w) }
func (g *timeoutGrid) Height() int32 { return int32(g.h) }
func (g *timeoutGrid) IsInside(x, y int32) bool {
	return x >= 0 && x < int32(g.w) && y >= 0 && y < int32(g.h)
}
func (g *timeoutGrid) IsWalkableAt(x, y int32) bool {
	return g.IsInside(x, y)
}
func (g *timeoutGrid) SetWalkableAt(x, y int32, v bool) {}
func (g *timeoutGrid) GetNodeAt(x, y int32) *Node       { return &g.nodes[int(y)*g.w+int(x)] }
func (g *timeoutGrid) GetNeighbors(n *Node, _ DiagonalMovement, buf []*Node) []*Node {
	out := buf[:0]
	for _, d := range [][2]int32{{0, -1}, {1, 0}, {0, 1}, {-1, 0}} {
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
			grid.nodes[y*4+x] = Node{X: int32(x), Y: int32(y), Walkable: true, Weight: 1}
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
