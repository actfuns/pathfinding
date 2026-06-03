package finder_test

import (
	"math/rand"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

// BenchmarkCompare compares A*, JPS, and JPS+ across grid sizes and obstacle densities.
// Run with: go test -bench=BenchmarkCompare -benchmem ./finder/
func BenchmarkCompare(b *testing.B) {
	sizes := []struct{ w, h int }{{100, 100}, {200, 200}}
	densities := []struct {
		name string
		pct  float64
	}{{"Open", 0}, {"Sparse", 0.1}, {"Dense", 0.2}}

	for _, sz := range sizes {
		for _, d := range densities {
			rng := rand.New(rand.NewSource(3))
			matrix := make([][]int, sz.h)
			for y := 0; y < sz.h; y++ {
				row := make([]int, sz.w)
				for x := 0; x < sz.w; x++ {
					if rng.Float64() < d.pct {
						row[x] = 1
					}
				}
				matrix[y] = row
			}
			base := grid.NewOrthogonalGrid(matrix)

			name := itoa(sz.w) + "x" + itoa(sz.h)

			b.Run(name+"/"+d.name+"/AStar", func(b *testing.B) {
				f := finder.NewAStarFinder()
				b.ReportAllocs()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					f.FindPath(0, 0, sz.w-1, sz.h-1, base)
				}
			})

			b.Run(name+"/"+d.name+"/JPS", func(b *testing.B) {
				f := finder.NewJumpPointFinder()
				b.ReportAllocs()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					f.FindPath(0, 0, sz.w-1, sz.h-1, base)
				}
			})

			b.Run(name+"/"+d.name+"/JPSPlus", func(b *testing.B) {
				f := finder.NewJPSPlusFinder()
				f.Precompute(base)
				b.ReportAllocs()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					f.FindPath(0, 0, sz.w-1, sz.h-1, base)
				}
			})
		}
	}
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	var buf [4]byte
	var i int
	for n > 0 {
		i++
		buf[4-i] = byte('0' + n%10)
		n /= 10
	}
	return string(buf[4-i:])
}
