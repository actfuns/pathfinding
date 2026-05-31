package pathfinding

// JumpPointFinder creates a JPS finder based on the diagonal movement setting.
func JumpPointFinder(opt *FinderOptions) Finder {
	if opt == nil {
		opt = &FinderOptions{}
	}
	switch opt.DiagonalMovement {
	case DiagonalNever:
		return NewJPFNeverMoveDiagonally(opt)
	case DiagonalAlways:
		return NewJPFAlwaysMoveDiagonally(opt)
	case DiagonalOnlyWhenNoObstacles:
		return NewJPFMoveDiagonallyIfNoObstacles(opt)
	default:
		return NewJPFMoveDiagonallyIfAtMostOneObstacle(opt)
	}
}
