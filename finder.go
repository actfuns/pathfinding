package pathfinding

// Finder is the interface for all pathfinding algorithms.
type Finder interface {
	FindPath(startX, startY, endX, endY int, grid *Grid) [][2]int
}
