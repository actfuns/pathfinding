package main

import (
	"fmt"
	"math"

	"github.com/actfuns/pathfinding/flowfield"
	"github.com/actfuns/pathfinding/grid"
)

type soldier struct {
	name   string
	x, y   float32 // world position (continuous, not snapped to tiles)
	speed  float32 // pixels per frame
	target [2]int  // current target tile
}

func main() {
	// Create a 10×8 grid with 32×32 px tiles
	matrix := grid.NewMatrix(10, 8)
	g := grid.NewOrthogonalGrid(matrix, grid.WithOrthogonalTileSize(32, 32))

	// Place obstacles
	g.SetWalkableAt(3, 3, false)
	g.SetWalkableAt(3, 4, false)
	g.SetWalkableAt(3, 5, false)

	// Goal in world coords (288,224) = tile (9,7)
	goalTX, goalTY := g.WorldToTile(288, 224)
	fmt.Printf("Goal world (288,224) → tile (%d,%d)\n\n", goalTX, goalTY)

	// Build the flow field once — all units share it
	ff := flowfield.New(g, goalTX, goalTY)

	// 5 soldiers with different speeds
	army := []*soldier{
		{"SoldierA", 32, 32, 12, [2]int{}},
		{"CavalryB", 64, 160, 24, [2]int{}},
		{"SoldierC", 224, 32, 10, [2]int{}},
		{"CavalryD", 160, 224, 20, [2]int{}},
		{"SoldierE", 32, 224, 14, [2]int{}},
	}

	// Initialise first target tile for each soldier
	for _, s := range army {
		tx, ty := g.WorldToTile(s.x, s.y)
		dx, dy := ff.GetDirection(tx, ty)
		s.target = [2]int{tx + dx, ty + dy}
	}

	fmt.Println("=== Movement (different speeds, smooth interpolation) ===")
	for frame := 1; frame <= 15; frame++ {
		fmt.Printf("--- Frame %d ---\n", frame)
		for _, s := range army {
			if s.speed == 0 {
				continue // arrived
			}

			// Centre of the target tile (world space)
			goalX, goalY := g.TileToWorld(s.target[0], s.target[1])

			// Distance to centre
			dx := goalX - s.x
			dy := goalY - s.y
			dist := float32(math.Sqrt(float64(dx*dx + dy*dy)))

			if dist < 1 {
				// Reached the target tile — get next direction
				ndx, ndy := ff.GetDirection(s.target[0], s.target[1])
				if ndx == 0 && ndy == 0 {
					fmt.Printf("  %s: 🏁 arrived! (speed=%.0f)\n", s.name, s.speed)
					s.speed = 0
					continue
				}
				s.target = [2]int{s.target[0] + ndx, s.target[1] + ndy}
				goalX, goalY = g.TileToWorld(s.target[0], s.target[1])
				dx = goalX - s.x
				dy = goalY - s.y
				dist = float32(math.Sqrt(float64(dx*dx + dy*dy)))
			}

			// Move toward target at this unit's speed
			move := s.speed
			if move > dist {
				move = dist
			}
			s.x += dx / dist * move
			s.y += dy / dist * move

			curTX, curTY := g.WorldToTile(s.x, s.y)
			fmt.Printf("  %s: world(%.1f,%.1f) tile(%d,%d) → target(%d,%d) dist=%.0fpx\n",
				s.name, s.x, s.y, curTX, curTY, s.target[0], s.target[1], dist)
		}
	}

	fmt.Println("\n=== Final positions ===")
	for _, s := range army {
		status := "moving"
		if s.speed == 0 {
			status = "arrived"
		}
		tx, ty := g.WorldToTile(s.x, s.y)
		fmt.Printf("  %s: world(%.1f,%.1f) tile(%d,%d) [%s]\n", s.name, s.x, s.y, tx, ty, status)
	}
}
