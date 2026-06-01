package finder

// DiagonalMovement represents allowed diagonal movement types.
type DiagonalMovement int

const (
	DiagonalAlways              DiagonalMovement = 1
	DiagonalNever               DiagonalMovement = 2
	DiagonalIfAtMostOneObstacle DiagonalMovement = 3
	DiagonalOnlyWhenNoObstacles DiagonalMovement = 4
)
