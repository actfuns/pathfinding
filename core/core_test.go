package core

import (
	"math"
	"testing"
)

// --- Grid ---

func TestNewGridFromMatrix(t *testing.T) {
	matrix := [][]int{
		{0, 1, 0},
		{1, 0, 1},
	}
	g := NewGrid(matrix)
	if g.Width != 3 {
		t.Errorf("expected width 3, got %d", g.Width)
	}
	if g.Height != 2 {
		t.Errorf("expected height 2, got %d", g.Height)
	}
	if !g.IsWalkableAt(0, 0) {
		t.Error("expected (0,0) walkable")
	}
	if g.IsWalkableAt(1, 0) {
		t.Error("expected (1,0) unwalkable")
	}
}

func TestNewGridFromDimensions(t *testing.T) {
	g := NewGrid(4, 5)
	if g.Width != 4 || g.Height != 5 {
		t.Errorf("expected 4x5, got %dx%d", g.Width, g.Height)
	}
	for y := 0; y < 5; y++ {
		for x := 0; x < 4; x++ {
			if !g.IsWalkableAt(x, y) {
				t.Errorf("expected all walkable, (%d,%d) is not", x, y)
			}
		}
	}
}

func TestGridIsInside(t *testing.T) {
	g := NewGrid([][]int{
		{0, 0},
		{0, 0},
	})
	tests := []struct {
		x, y int
		want bool
	}{
		{0, 0, true},
		{1, 1, true},
		{-1, 0, false},
		{0, -1, false},
		{2, 0, false},
		{0, 2, false},
	}
	for _, tt := range tests {
		got := g.IsInside(tt.x, tt.y)
		if got != tt.want {
			t.Errorf("IsInside(%d,%d) = %v, want %v", tt.x, tt.y, got, tt.want)
		}
	}
}

func TestGridSetWalkableAt(t *testing.T) {
	g := NewGrid([][]int{{0}})
	if !g.IsWalkableAt(0, 0) {
		t.Fatal("expected walkable initially")
	}
	g.SetWalkableAt(0, 0, false)
	if g.IsWalkableAt(0, 0) {
		t.Error("expected unwalkable after set")
	}
}

func TestGridGetNodeAt(t *testing.T) {
	g := NewGrid([][]int{{0, 1}})
	n := g.GetNodeAt(1, 0)
	if n == nil {
		t.Fatal("expected node")
	}
	if n.X != 1 || n.Y != 0 || n.Walkable {
		t.Error("expected node at (1,0) unwalkable")
	}
}

func TestGridGetNeighborsDiagonalNever(t *testing.T) {
	g := NewGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})
	node := g.GetNodeAt(1, 1)
	neighbors := g.GetNeighbors(node, DiagonalNever)
	if len(neighbors) != 4 {
		t.Errorf("expected 4 neighbors (no diag), got %d", len(neighbors))
	}
}

func TestGridGetNeighborsDiagonalAlways(t *testing.T) {
	g := NewGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})
	node := g.GetNodeAt(1, 1)
	neighbors := g.GetNeighbors(node, DiagonalAlways)
	if len(neighbors) != 8 {
		t.Errorf("expected 8 neighbors, got %d", len(neighbors))
	}
}

func TestGridGetNeighborsDiagonalOnlyWhenNoObstacles(t *testing.T) {
	// (1,0) is wall, but both straight neighbors around (0,0) that are
	// inside bounds and walkable: right(1,0)=wall. So diag should not be allowed.
	g := NewGrid([][]int{
		{0, 1, 0},
		{0, 0, 0},
		{0, 0, 0},
	})
	node := g.GetNodeAt(1, 1)
	neighbors := g.GetNeighbors(node, DiagonalOnlyWhenNoObstacles)
	// From (1,1): up(1,0)=wall, right(2,0)=walkable, down(1,2)=walkable, left(0,1)=walkable
	// NE: up right both need to be walkable: up is wall → no
	// SE: right AND down both walkable → yes (2,2)
	// SW: down AND left both walkable → yes (0,2)
	// NW: left AND up: up is wall → no
	hasSE, hasSW := false, false
	for _, n := range neighbors {
		if n.X == 2 && n.Y == 2 {
			hasSE = true
		}
		if n.X == 0 && n.Y == 2 {
			hasSW = true
		}
	}
	if !hasSE || !hasSW {
		t.Error("expected SE(2,2) and SW(0,2) via OnlyWhenNoObstacles")
	}
	// NE(2,0) and NW(0,0) should NOT be reachable since (1,0) is wall
	for _, n := range neighbors {
		if n.X == 2 && n.Y == 0 {
			t.Error("NE(2,0) should not be reachable when up is wall")
		}
		if n.X == 0 && n.Y == 0 {
			t.Error("NW(0,0) should not be reachable when up is wall")
		}
	}
}

func TestGridGetNeighborsDiagonalIfAtMostOneObstacle(t *testing.T) {
	g := NewGrid([][]int{
		{1, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})
	// (0,0) is wall, start from (1,0)
	node := g.GetNodeAt(1, 0)
	neighbors := g.GetNeighbors(node, DiagonalIfAtMostOneObstacle)
	// From (1,0): left(0,0)=wall, right(2,0)=walkable, up(-1)=OOB, down(1,1)=walkable
	// NW diag from (1,0): needs left OR up. left wall→up OOB→d0=false → no NW
	// Actually d0 = s3||s0 where s3=left, s0=up. s3 is wall→false, s0 OOB→false → d0=false
	// NE diag(2,-1): OOB
	// SE diag(2,1): s1||s2 = right||down = true→ d2=true, target walkable → should have (2,1)
	hasSE := false
	for _, n := range neighbors {
		if n.X == 2 && n.Y == 1 {
			hasSE = true
		}
	}
	if !hasSE {
		t.Error("expected SE (2,1) via IfAtMostOneObstacle")
	}
}

func TestGridClone(t *testing.T) {
	g := NewGrid([][]int{
		{0, 1},
		{1, 0},
	})
	c := g.Clone()
	if c.Width != 2 || c.Height != 2 {
		t.Errorf("expected 2x2, got %dx%d", c.Width, c.Height)
	}
	if c.IsWalkableAt(0, 0) != g.IsWalkableAt(0, 0) {
		t.Error("clone walkability mismatch")
	}
	// Mutating clone shouldn't affect original
	c.SetWalkableAt(0, 0, false)
	if !g.IsWalkableAt(0, 0) {
		t.Error("original should still be walkable after clone mutation")
	}
}

// --- Node ---

func TestNewNode(t *testing.T) {
	n := NewNode(3, 5)
	if n.X != 3 || n.Y != 5 {
		t.Errorf("expected (3,5), got (%d,%d)", n.X, n.Y)
	}
	if !n.Walkable {
		t.Error("expected walkable by default")
	}
}

// --- MinHeap ---

func TestMinHeapPushPop(t *testing.T) {
	h := NewMinHeap(func(a, b *Node) bool { return a.F < b.F })
	h.Push(&Node{F: 3})
	h.Push(&Node{F: 1})
	h.Push(&Node{F: 2})

	if h.Len() != 3 {
		t.Errorf("expected len 3, got %d", h.Len())
	}
	if h.Empty() {
		t.Error("expected non-empty")
	}

	if n := h.Pop(); n.F != 1 {
		t.Errorf("expected F=1, got %v", n.F)
	}
	if n := h.Pop(); n.F != 2 {
		t.Errorf("expected F=2, got %v", n.F)
	}
	if n := h.Pop(); n.F != 3 {
		t.Errorf("expected F=3, got %v", n.F)
	}
	if !h.Empty() {
		t.Error("expected empty after all pops")
	}
}

func TestMinHeapUpdateItem(t *testing.T) {
	h := NewMinHeap(func(a, b *Node) bool { return a.F < b.F })
	n1 := &Node{F: 10}
	n2 := &Node{F: 20}
	n3 := &Node{F: 30}
	h.Push(n1)
	h.Push(n2)
	h.Push(n3)

	// Decrease key of n3
	n3.F = 5
	h.UpdateItem(n3)

	if n := h.Pop(); n.F != 5 {
		t.Errorf("expected F=5, got %v", n.F)
	}
}

func TestMinHeapEmptyPop(t *testing.T) {
	h := NewMinHeap(func(a, b *Node) bool { return a.F < b.F })
	// Pop from an empty heap panics in the current implementation.
	// This is acceptable since the heap is used internally by finders
	// where Empty() is always checked before Pop().
	defer func() {
		if r := recover(); r == nil {
			t.Error("expected panic on pop from empty heap")
			_ = h
		}
	}()
	h.Pop()
}

// --- Heuristic ---

func TestManhattan(t *testing.T) {
	if d := Manhattan(3, 4); d != 7 {
		t.Errorf("Manhattan(3,4) = %v, want 7", d)
	}
	if d := Manhattan(0, 0); d != 0 {
		t.Errorf("Manhattan(0,0) = %v, want 0", d)
	}
}

func TestEuclidean(t *testing.T) {
	if d := Euclidean(3, 4); d != 5 {
		t.Errorf("Euclidean(3,4) = %v, want 5", d)
	}
}

func TestOctile(t *testing.T) {
	// dx=dy=10: Octile = (sqrt2-1)*10 + 10 = 14.1421...
	d := Octile(10, 10)
	expected := (SQRT2-1)*10 + 10
	if math.Abs(d-expected) > 1e-10 {
		t.Errorf("Octile(10,10) = %v, want %v", d, expected)
	}
	// dx < dy: should use f*dx + dy
	d2 := Octile(5, 10)
	expected2 := (SQRT2-1)*5 + 10
	if math.Abs(d2-expected2) > 1e-10 {
		t.Errorf("Octile(5,10) = %v, want %v", d2, expected2)
	}
}

func TestChebyshev(t *testing.T) {
	if d := Chebyshev(3, 7); d != 7 {
		t.Errorf("Chebyshev(3,7) = %v, want 7", d)
	}
	if d := Chebyshev(7, 3); d != 7 {
		t.Errorf("Chebyshev(7,3) = %v, want 7", d)
	}
}

// --- Path utilities ---

func TestBacktrace(t *testing.T) {
	n1 := &Node{X: 0, Y: 0}
	n2 := &Node{X: 1, Y: 0, Parent: n1}
	n3 := &Node{X: 1, Y: 1, Parent: n2}
	path := Backtrace(n3)
	expected := [][2]int{{0, 0}, {1, 0}, {1, 1}}
	if !pathEqual(path, expected) {
		t.Errorf("got %v, want %v", path, expected)
	}
}

func TestBacktraceNil(t *testing.T) {
	path := Backtrace(nil)
	if len(path) != 0 {
		t.Errorf("expected empty path for nil, got %v", path)
	}
}

func TestBiBacktrace(t *testing.T) {
	// Simple chain: A ← B ← C ← D
	n1 := &Node{X: 0, Y: 0}
	n2 := &Node{X: 1, Y: 0, Parent: n1}
	n3 := &Node{X: 2, Y: 0, Parent: n2}
	_ = n3

	// BiBacktrace from two separate chains
	startA := &Node{X: 0, Y: 0}
	startB := &Node{X: 1, Y: 0, Parent: startA}

	endA := &Node{X: 4, Y: 4}
	endB := &Node{X: 3, Y: 4, Parent: endA}

	path := BiBacktrace(startB, endB)
	// startPath from startB: {0,0}→{1,0}, reversed: {1,0}→{0,0}
	// endPath from endB: {4,4}→{3,4}
	// result: {1,0}, {0,0}, {4,4}, {3,4}
	if len(path) != 4 {
		t.Errorf("expected 4 points, got %d: %v", len(path), path)
	}
}

func TestPathLength(t *testing.T) {
	path := [][2]int{{0, 0}, {3, 0}, {3, 4}}
	l := PathLength(path)
	expected := 3.0 + 4.0
	if l != expected {
		t.Errorf("expected %v, got %v", expected, l)
	}
}

func TestPathLengthEmpty(t *testing.T) {
	if l := PathLength(nil); l != 0 {
		t.Errorf("expected 0 for empty, got %v", l)
	}
	if l := PathLength([][2]int{{0, 0}}); l != 0 {
		t.Errorf("expected 0 for single point, got %v", l)
	}
}

func TestInterpolate(t *testing.T) {
	// Horizontal line
	line := Interpolate(0, 0, 3, 0)
	if len(line) != 4 {
		t.Errorf("expected 4 points, got %d: %v", len(line), line)
	}
	if line[0] != [2]int{0, 0} || line[3] != [2]int{3, 0} {
		t.Error("wrong endpoints")
	}

	// Single point
	line = Interpolate(2, 2, 2, 2)
	if len(line) != 1 || line[0] != [2]int{2, 2} {
		t.Error("expected single point")
	}
}

func TestExpandPath(t *testing.T) {
	path := [][2]int{{0, 0}, {2, 0}, {2, 2}}
	expanded := ExpandPath(path)
	if len(expanded) != 5 {
		t.Errorf("expected 5 points, got %d: %v", len(expanded), expanded)
	}
	if expanded[0] != [2]int{0, 0} || expanded[len(expanded)-1] != [2]int{2, 2} {
		t.Error("wrong endpoints")
	}
}

func TestExpandPathShort(t *testing.T) {
	if r := ExpandPath(nil); r != nil {
		t.Error("expected nil for nil path")
	}
	if r := ExpandPath([][2]int{{0, 0}}); r != nil {
		t.Error("expected nil for single point")
	}
}

func TestSmoothenPath(t *testing.T) {
	// Open grid, path can be smoothed
	grid := NewGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})
	path := [][2]int{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}}
	smoothed := SmoothenPath(grid, path)
	// All on same row, no obstacles → should be {0,0}, {4,0}
	if len(smoothed) != 2 {
		t.Errorf("expected 2 points, got %d: %v", len(smoothed), smoothed)
	}

	// Slightly jagged path: (0,0)→(2,0)→(2,3)
	// Line from (0,0) to (2,3) via Bresenham: should skip (2,0)
	path2 := [][2]int{{0, 0}, {2, 0}, {2, 3}}
	smoothed2 := SmoothenPath(grid, path2)
	if len(smoothed2) < 2 {
		t.Errorf("expected at least 2 points, got %d: %v", len(smoothed2), smoothed2)
	}
	if smoothed2[0] != [2]int{0, 0} || smoothed2[len(smoothed2)-1] != [2]int{2, 3} {
		t.Error("wrong endpoints")
	}
}

func TestCompressPath(t *testing.T) {
	path := [][2]int{{0, 0}, {2, 0}, {4, 0}, {4, 2}, {4, 4}}
	compressed := CompressPath(path)
	// {0,0}→{2,0}→{4,0} same direction, so {2,0} removed
	// {4,0}→{4,2}→{4,4} same direction, so {4,2} removed
	if len(compressed) != 3 {
		t.Errorf("expected 3 points, got %d: %v", len(compressed), compressed)
	}
	if compressed[0] != [2]int{0, 0} || compressed[1] != [2]int{4, 0} || compressed[2] != [2]int{4, 4} {
		t.Error("wrong compressed path")
	}
}

func TestCompressPathShort(t *testing.T) {
	if r := CompressPath(nil); r != nil {
		t.Error("expected nil for nil")
	}
	r := CompressPath([][2]int{{0, 0}, {1, 1}})
	if len(r) != 2 {
		t.Errorf("expected same path for length<3, got %v", r)
	}
}

// --- Math utilities ---

func TestAbsInt(t *testing.T) {
	if AbsInt(5) != 5 {
		t.Error("AbsInt(5)")
	}
	if AbsInt(-5) != 5 {
		t.Error("AbsInt(-5)")
	}
	if AbsInt(0) != 0 {
		t.Error("AbsInt(0)")
	}
}

func TestMaxInt(t *testing.T) {
	if MaxInt(3, 7) != 7 {
		t.Error("MaxInt(3,7)")
	}
	if MaxInt(7, 3) != 7 {
		t.Error("MaxInt(7,3)")
	}
	if MaxInt(5, 5) != 5 {
		t.Error("MaxInt(5,5)")
	}
}

// --- DiagonalMovement constants ---

func TestDiagonalMovementValues(t *testing.T) {
	if DiagonalAlways != 1 {
		t.Errorf("DiagonalAlways = %d", DiagonalAlways)
	}
	if DiagonalNever != 2 {
		t.Errorf("DiagonalNever = %d", DiagonalNever)
	}
	if DiagonalIfAtMostOneObstacle != 3 {
		t.Errorf("DiagonalIfAtMostOneObstacle = %d", DiagonalIfAtMostOneObstacle)
	}
	if DiagonalOnlyWhenNoObstacles != 4 {
		t.Errorf("DiagonalOnlyWhenNoObstacles = %d", DiagonalOnlyWhenNoObstacles)
	}
}

// --- helper ---

func pathEqual(a, b [][2]int) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
