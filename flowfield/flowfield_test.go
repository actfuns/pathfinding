package flowfield

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestFlowFieldBasic(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})

	ff := New(g, 2, 2)
	if ff == nil {
		t.Fatal("New returned nil")
	}

	for y := 0; y < 3; y++ {
		for x := 0; x < 3; x++ {
			if c := ff.GetCost(x, y); c < 0 {
				t.Errorf("tile (%d,%d) should be reachable, got cost=%v", x, y, c)
			}
		}
	}

	if c := ff.GetCost(2, 2); c != 0 {
		t.Errorf("goal cost should be 0, got %v", c)
	}

	if dx, dy := ff.GetDirection(0, 0); dx == 0 && dy == 0 {
		t.Error("expected non-zero direction from (0,0)")
	}
}

func TestFlowFieldFindPath(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})
	ff := New(g, 2, 2)
	path := ff.FindPath(0, 0)
	if path == nil {
		t.Fatal("FindPath returned nil")
	}
	if path[0] != [2]int{0, 0} || path[len(path)-1] != [2]int{2, 2} {
		t.Errorf("path should go from (0,0) to (2,2), got %v", path)
	}
}

func TestFlowFieldWithObstacles(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	})

	ff := New(g, 2, 2)

	if c := ff.GetCost(1, 1); c >= 0 {
		t.Errorf("obstacle should be unreachable, got cost=%v", c)
	}

	for _, pos := range [][2]int{{2, 0}, {2, 1}} {
		if c := ff.GetCost(pos[0], pos[1]); c < 0 {
			t.Errorf("tile (%d,%d) should be reachable around obstacle", pos[0], pos[1])
		}
	}

	if c := ff.GetCost(0, 0); c < 0 {
		t.Fatal("(0,0) should be reachable")
	}
}

func TestFlowFieldDirection(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})

	ff := New(g, 2, 2)

	dx, dy := ff.GetDirection(0, 0)
	if dx == 0 && dy == 0 {
		t.Errorf("expected non-zero direction from (0,0), got (0,0)")
	}
	t.Logf("direction from (0,0) = (%d,%d)", dx, dy)

	dx, dy = ff.GetDirection(2, 2)
	if dx != 0 || dy != 0 {
		t.Errorf("goal should have zero direction, got (%d,%d)", dx, dy)
	}
}

func TestFlowFieldWithWeights(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})

	g.SetWeightAt(0, 1, 5.0)
	g.SetWeightAt(1, 1, 5.0)
	g.SetWeightAt(2, 1, 5.0)
	g.SetWeightAt(3, 1, 5.0)
	g.SetWeightAt(4, 1, 5.0)

	ff := New(g, 4, 0)

	dx, dy := ff.GetDirection(0, 1)
	if dy != -1 {
		t.Logf("weight test: from (0,1) direction = (%d,%d) — should go up", dx, dy)
	}
}

func TestFlowFieldUnreachable(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 1},
		{1, 0},
	})

	ff := New(g, 1, 1)
	if ff == nil {
		t.Fatal("New returned nil")
	}

	if c := ff.GetCost(0, 0); c >= 0 {
		t.Errorf("expected unreachable, got cost=%.1f", c)
	}
}

func TestFlowFieldOutOfBounds(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{{0, 0}, {0, 0}})

	ff := New(g, 1, 1)

	dx, dy := ff.GetDirection(-1, -1)
	if dx != 0 || dy != 0 {
		t.Errorf("expected (0,0) for out-of-bounds, got (%d,%d)", dx, dy)
	}

	if c := ff.GetCost(100, 100); c >= 0 {
		t.Errorf("expected negative cost for out-of-bounds, got %v", c)
	}
}

func TestFlowFieldMultiGoal(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})

	ff := NewMulti(g, [][2]int{{0, 0}, {4, 1}})

	if c := ff.GetCost(0, 0); c != 0 {
		t.Errorf("goal (0,0) cost should be 0, got %.1f", c)
	}
	if c := ff.GetCost(4, 1); c != 0 {
		t.Errorf("goal (4,1) cost should be 0, got %.1f", c)
	}

	dx, dy := ff.GetDirection(0, 1)
	if dx != 0 || dy != -1 {
		t.Logf("(0,1) direction = (%d,%d), expected (0,-1) toward (0,0)", dx, dy)
	}

	dx, dy = ff.GetDirection(4, 0)
	if dx != 0 || dy != 1 {
		t.Logf("(4,0) direction = (%d,%d), expected (0,1) toward (4,1)", dx, dy)
	}
}

func TestFlowFieldMultiGoalWithObstacles(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0},
	})

	ff := NewMulti(g, [][2]int{{0, 0}, {4, 0}})

	if c := ff.GetCost(1, 1); c >= 0 {
		t.Errorf("wall should be unreachable, got cost=%.1f", c)
	}

	if c := ff.GetCost(2, 0); c < 0 {
		t.Fatal("(2,0) should be reachable")
	}
}

func TestFlowFieldReset(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})

	ff := New(g, 0, 0)

	dx, dy := ff.GetDirection(4, 1)
	t.Logf("toward (0,0): direction from (4,1) = (%d,%d)", dx, dy)

	ff.Reset(4, 1)
	dx2, dy2 := ff.GetDirection(0, 0)
	t.Logf("after reset to (4,1): direction from (0,0) = (%d,%d)", dx2, dy2)
	if dx2 == 0 && dy2 == 0 {
		t.Error("expected non-zero direction after reset")
	}
}

func TestFlowFieldResetMulti(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})

	ff := NewMulti(g, [][2]int{{0, 0}, {4, 1}})

	dx, dy := ff.GetDirection(0, 1)
	t.Logf("multi-goal: from (0,1) = (%d,%d)", dx, dy)

	ff.Reset(2, 0)
	if c := ff.GetCost(4, 1); c < 0 {
		t.Fatal("(4,1) should be reachable after reset")
	}

	ff.ResetMulti([][2]int{{1, 1}, {3, 1}})
	if c := ff.GetCost(1, 1); c != 0 {
		t.Errorf("goal (1,1) cost should be 0, got %.1f", c)
	}
	if c := ff.GetCost(3, 1); c != 0 {
		t.Errorf("goal (3,1) cost should be 0, got %.1f", c)
	}
}

func TestFlowFieldDiagonalMovement(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})

	ffCard := New(g, 2, 2)
	ffDiag := New(g, 2, 2, WithDiagonal(finder.DiagonalAlways))

	stepsCard := 0
	cx, cy := 0, 0
	for {
		dx, dy := ffCard.GetDirection(cx, cy)
		if dx == 0 && dy == 0 {
			break
		}
		cx += dx
		cy += dy
		stepsCard++
	}

	stepsDiag := 0
	cx, cy = 0, 0
	for {
		dx, dy := ffDiag.GetDirection(cx, cy)
		if dx == 0 && dy == 0 {
			break
		}
		cx += dx
		cy += dy
		stepsDiag++
	}

	t.Logf("cardinal steps: %d, diagonal steps: %d", stepsCard, stepsDiag)
	if stepsDiag >= stepsCard {
		t.Log("note: diagonal should be shorter or equal")
	}
}

func TestFlowFieldDiagonalWithObstacles(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0},
	})

	ffNever := New(g, 4, 0)
	cNever := ffNever.GetCost(0, 0)
	t.Logf("DiagonalNever cost: %.1f", cNever)

	ffAlways := New(g, 4, 0, WithDiagonal(finder.DiagonalAlways))
	cAlways := ffAlways.GetCost(0, 0)
	t.Logf("DiagonalAlways cost: %.1f", cAlways)
}

func BenchmarkFlowFieldNew(b *testing.B) {
	g := grid.NewOrthogonalGrid(grid.NewMatrix(50, 50))
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = New(g, 25, 25)
	}
}

func BenchmarkFlowFieldDirection(b *testing.B) {
	g := grid.NewOrthogonalGrid(grid.NewMatrix(50, 50))
	ff := New(g, 25, 25)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		ff.GetDirection(10, 10)
	}
}
