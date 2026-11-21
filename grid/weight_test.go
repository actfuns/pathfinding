package grid

import (
	"math"
	"testing"
)

func TestSetWeightAtRejectsInvalid(t *testing.T) {
	grids := []struct {
		name string
		g    Grid
	}{
		{"orthogonal", NewOrthogonalGrid(NewMatrix(2, 2))},
		{"hex", NewHexGrid(NewMatrix(2, 2))},
		{"staggered", NewStaggeredGrid(NewMatrix(2, 2))},
	}
	invalid := []float32{0, -1, float32(math.NaN()), float32(math.Inf(1)), float32(math.Inf(-1))}

	for _, tc := range grids {
		for _, w := range invalid {
			func() {
				defer func() {
					if recover() == nil {
						t.Errorf("%s: SetWeightAt(%v) did not panic", tc.name, w)
					}
				}()
				tc.g.(MutableGrid).SetWeightAt(0, 0, w)
			}()
			if got := tc.g.GetWeightAt(0, 0); got != 1 {
				t.Errorf("%s: weight changed to %v after rejecting %v", tc.name, got, w)
			}
		}

		tc.g.(MutableGrid).SetWeightAt(0, 0, 2.5)
		if got := tc.g.GetWeightAt(0, 0); got != 2.5 {
			t.Errorf("%s: weight = %v, want 2.5", tc.name, got)
		}
		if !tc.g.IsWalkableAt(0, 0) {
			t.Errorf("%s: setting a weight must not change walkability", tc.name)
		}
	}
}

func TestTileWeightBoundsCountsSharedMinimum(t *testing.T) {
	g := NewOrthogonalGrid(NewMatrix(4, 1)).(*OrthogonalGrid)
	g.SetWeightAt(0, 0, 0.25)
	g.SetWeightAt(1, 0, 0.25)
	g.SetWeightAt(2, 0, 0.5)
	g.SetWeightAt(0, 0, 1)
	if g.weightBounds.count != 1 || g.HeuristicWeightLowerBound() != 0.25 {
		t.Fatalf("unexpected bounds after removing the first minimum: %+v", g.weightBounds)
	}
	cloned, err := g.Clone(func(MutableGrid) {})
	if err != nil {
		t.Fatal(err)
	}
	cloned.(*OrthogonalGrid).SetWeightAt(1, 0, 1)
	if cloned.(*OrthogonalGrid).HeuristicWeightLowerBound() != 0.5 || g.HeuristicWeightLowerBound() != 0.25 {
		t.Fatal("raising the last minimum on a clone must update only the clone")
	}
	g.SetWeightAt(1, 0, 1)
	g.SetWeightAt(2, 0, 1)
	if g.HeuristicWeightLowerBound() != 1 {
		t.Fatalf("got %g, want the default bound", g.HeuristicWeightLowerBound())
	}
}
