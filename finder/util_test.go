package finder

import (
	"slices"
	"testing"
)

func TestExpandPathPreservesInterpolationAtJoins(t *testing.T) {
	path := [][2]int32{{0, 0}, {3, 1}, {3, 1}, {2, 3}}
	want := [][2]int32{{0, 0}, {1, 0}, {2, 1}, {3, 1}, {3, 2}, {2, 3}}
	if got := ExpandPath(path); !slices.Equal(got, want) {
		t.Fatalf("expanded path = %v, want %v", got, want)
	}
	if got := ExpandPath(path[:1]); got != nil {
		t.Fatalf("singleton compressed path = %v, want nil", got)
	}
	if got := Interpolate(3, 1, 3, 1); !slices.Equal(got, path[1:2]) {
		t.Fatalf("singleton line = %v, want %v", got, path[1:2])
	}
}
