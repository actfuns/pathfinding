package finder

// Finder is the interface for all pathfinding algorithms.
//
// The returned [][2]int is backed by an internal buffer and is only valid
// until the next FindPath call on the same Finder. If you need to persist
// the result, copy it (e.g., append([][2]int{}, path...)).
type Finder interface {
	FindPath(startX, startY, endX, endY int, grid Grid) [][2]int
}
